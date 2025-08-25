import string
import cupy as cp

# 此文件定义了使用CuPy实现的自定义CUDA内核，用于加速断层扫描图的生成和处理。

def utils_point(resolution, n_row, n_col):
    """
    生成一个包含点处理相关工具函数的CUDA C++代码字符串。
    这些函数主要用于将世界坐标系中的点映射到地图的栅格索引。

    Args:
        resolution (float): 地图分辨率。
        n_row (int): 地图的行数。
        n_col (int): 地图的列数。

    Returns:
        str: 包含CUDA C++工具函数的代码字符串。
    """
    util_preamble = string.Template(
        '''
        // 设备端函数：将一维坐标转换为栅格索引
        __device__ int getIndexLine(float16 x, float16 center)
        {
            int i = round((x - center) / ${resolution});
            return i;
        }

        // 设备端函数：将二维点(x, y)转换为层内的一维索引
        __device__ int getIndexMap_1d(float16 x, float16 y, float16 cx, float16 cy)
        {
            // 计算x和y方向的栅格索引
            int idx_x = getIndexLine(x, cx) + ${n_row} / 2;
            int idx_y = getIndexLine(y, cy) + ${n_col} / 2;

            // 检查索引是否在地图范围内
            if (idx_x < 0 || idx_x >= ${n_row} || idx_y < 0 || idx_y >= ${n_col})
            {
                return -1; // 超出范围返回-1
            }
            // 返回一维索引
            return ${n_col} * idx_x + idx_y;
        }

        // 设备端函数：获取点在多层地图块中的一维索引
        __device__ int getIndexBlock_1d(int idx, int layer_n)
        {
            return (int)${layer_size} * layer_n + idx;
        }

        // 设备端函数：原子操作，计算浮点数的最大值
        __device__ static float atomicMaxFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fmaxf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }

        // 设备端函数：原子操作，计算浮点数的最小值
        __device__ static float atomicMinFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fminf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }
        '''
    ).substitute(
        resolution=resolution,
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def utils_map(n_row, n_col):
    """
    生成一个包含地图处理相关工具函数的CUDA C++代码字符串。
    这些函数主要用于在地图栅格上进行邻域操作。

    Args:
        n_row (int): 地图的行数。
        n_col (int): 地图的列数。

    Returns:
        str: 包含CUDA C++工具函数的代码字符串。
    """
    util_preamble=string.Template(
        '''
        // 设备端函数：获取相对位置(dx, dy)处的栅格的一维索引
        __device__ int getIdxRelative(int idx, int dx, int dy) 
        {
            // 从一维索引反推二维坐标
            int idx_2d = idx % (int)${layer_size};
            int idx_x = idx_2d / ${n_col};
            int idx_y = idx_2d % ${n_col};
            // 计算相对坐标
            int idx_rx = idx_x + dx;
            int idx_ry = idx_y + dy;

            // 检查相对坐标是否越界
            if ( idx_rx < 0 || idx_rx > (${n_row} - 1) ) 
                return -1;
            if ( idx_ry < 0 || idx_ry > (${n_col} - 1) )
                return -1;

            // 返回新的一维索引
            return ${n_col} * dx + dy + idx;
        }
        '''
    ).substitute(
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def tomographyKernel(resolution, n_row, n_col, n_slice, slice_h0, slice_dh):
    """
    定义并返回一个CuPy ElementwiseKernel，用于从点云生成断层扫描图。

    对于每个点，内核会判断它属于哪个切片，并更新该切片对应栅格的地面高度(layers_g)和
    天花板高度(layers_c)。

    Args:
        resolution (float): 地图分辨率。
        n_row (int): 地图行数。
        n_col (int): 地图列数。
        n_slice (int): 切片数量。
        slice_h0 (float): 初始切片的高度。
        slice_dh (float): 切片之间的高度差。

    Returns:
        cupy.ElementwiseKernel: 用于断层扫描图生成的CUDA内核。
    """
    tomography_kernel = cp.ElementwiseKernel(
        in_params='raw U points, raw U center',
        out_params='raw U layers_g, raw U layers_c',
        preamble=utils_point(resolution, n_row, n_col),
        operation=string.Template(
            '''
            // 获取点的x, y, z坐标
            U px = points[i * 3];
            U py = points[i * 3 + 1];
            U pz = points[i * 3 + 2];

            // 将点(px, py)转换为地图上的一维索引
            int idx = getIndexMap_1d(px, py, center[0], center[1]);
            if ( idx < 0 ) // 如果点在地图外，则跳过
                return; 

            // 遍历所有切片层
            for ( int s_idx = 0; s_idx < ${n_slice}; s_idx ++ )
            {
                U slice = ${slice_h0} + s_idx * ${slice_dh};
                // 如果点的高度低于或等于当前切片，更新地面高度（取最大值）
                if ( pz <= slice )
                    atomicMaxFloat(&layers_g[getIndexBlock_1d(idx, s_idx)], pz);
                // 否则，更新天花板高度（取最小值）
                else
                    atomicMinFloat(&layers_c[getIndexBlock_1d(idx, s_idx)], pz);
            }
            '''
        ).substitute(
            n_slice=n_slice,
            slice_h0=slice_h0,
            slice_dh=slice_dh
        ),
        name='tomography_kernel'
    )
                            
    return tomography_kernel


def travKernel(
    n_row, n_col, half_kernel_size,
    interval_min, interval_free, step_cross, step_stand, standable_th, cost_barrier
    ):
    """
    定义并返回一个CuPy ElementwiseKernel，用于计算可通行性成本。

    此内核根据地形的几何特征（如垂直间隙、坡度）来评估每个栅格的通行成本。

    Args:
        n_row, n_col: 地图尺寸。
        half_kernel_size: 分析邻域的半核大小。
        interval_min, interval_free: 最小和自由通行垂直间隙。
        step_cross, step_stand: 可跨越和可站立的台阶高度。
        standable_th: 可站立邻域的阈值。
        cost_barrier: 障碍物的成本。

    Returns:
        cupy.ElementwiseKernel: 用于计算可通行性成本的CUDA内核。
    """
    trav_kernel = cp.ElementwiseKernel(
        in_params='raw U interval, raw U grad_mag_sq, raw U grad_mag_max',
        out_params='raw U trav_cost',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            // 检查垂直间隙是否小于最小要求
            if ( interval[i] < ${interval_min} )
            {
                trav_cost[i] = ${cost_barrier}; // 设为障碍物
                return;
            }
            else // 间隙成本
                trav_cost[i] += max(0.0, 20 * (${interval_free} - interval[i]));

            // 检查坡度是否平缓（可站立）
            if ( grad_mag_sq[i] <= ${step_stand_sq} )
            {
                trav_cost[i] += 15 * grad_mag_sq[i] / ${step_stand_sq}; // 坡度成本
                return;
            }
            else // 坡度较陡
            {
                // 检查邻域最大坡度是否在可跨越范围内
                if ( grad_mag_max[i] <= ${step_cross_sq} )
                {
                    // 检查周围是否有足够的可站立区域
                    int standable_grids = 0;
                    for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ ) 
                    {
                        for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ ) 
                        {
                            int idx = getIdxRelative(i, dx, dy);
                            if ( idx < 0 )
                                continue;
                            if ( grad_mag_sq[idx] < ${step_stand_sq} )
                                standable_grids += 1;
                        }
                    }
                    if ( standable_grids < ${standable_th} )
                    {
                        trav_cost[i] = ${cost_barrier}; // 设为障碍物
                        return;
                    }
                    else // 崎岖地形成本
                        trav_cost[i] += 20 * grad_mag_max[i] / ${step_cross_sq};
                }
                else // 坡度过大，不可通行
                {
                    trav_cost[i] = ${cost_barrier};
                    return;
                }
            } 
            '''
        ).substitute(
            half_kernel_size=half_kernel_size,
            interval_min=interval_min,
            interval_free=interval_free,
            step_cross_sq=step_cross ** 2,
            step_stand_sq=step_stand ** 2,
            standable_th=standable_th,
            cost_barrier=cost_barrier
        ),
        name='trav_kernel'
    )
                            
    return trav_kernel


def inflationKernel(n_row, n_col, half_kernel_size):
    """
    定义并返回一个CuPy ElementwiseKernel，用于对成本地图进行膨胀操作。

    此内核通过考虑邻域的成本来“膨胀”障碍物，为路径规划增加安全边际。

    Args:
        n_row, n_col: 地图尺寸。
        half_kernel_size: 膨胀核的半大小。

    Returns:
        cupy.ElementwiseKernel: 用于成本地图膨胀的CUDA内核。
    """
    inflation_kernel = cp.ElementwiseKernel(
        in_params='raw U trav_cost, raw U score_table',
        out_params='raw U inflated_cost',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            int counter = 0;
            // 遍历邻域
            for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ ) 
            {
                for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ ) 
                {
                    int idx = getIdxRelative(i, dx, dy);
                    if ( idx >= 0 )
                        // 更新当前栅格的膨胀成本为邻域成本加权后的最大值
                        inflated_cost[i] = max(inflated_cost[i], trav_cost[idx] * score_table[counter]);
                    counter += 1;
                }
            }
            '''
        ).substitute(
            half_kernel_size=half_kernel_size
        ),
        name='inflation_kernel'
    )
                            
    return inflation_kernel