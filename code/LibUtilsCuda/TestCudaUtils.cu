
#include <LibCudaUtils/CudaUtils.hxx>
#include <LibCudaUtils/CudaMemory.h>

#include <LibCudaUtils/culaut/culaut.hxx>
#include <LibCudaUtils/culaut/xprojectionmatrix.hxx>

#include <Utils/TimerWin32.hxx>

__global__ void kernel_pinv_float(int n, float *Ps)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n) return;
	float *P=Ps+12*idx;
	culaut::projection_matrix_pseudoinverse_transpose_inplace(P);
}

__global__ void kernel_null_float(int n, float *Ps, float *Cs)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n) return;
	float *P=Ps+12*idx;
	culaut::projection_matrix_source_position(P,Cs+idx*4);
}

__global__ void kernel_null_pinv_float(int n, float *Ps, float *Cs)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n) return;
	float *P=Ps+12*idx;
	culaut::projection_matrix_source_position(P,Cs+idx*4);
	culaut::projection_matrix_pseudoinverse_transpose_inplace(P);
}

//////////////////////////////////////////////////

__global__ void kernel_pinv_double(int n, double *Ps)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n) return;
	double *P=Ps+12*idx;
	culaut::projection_matrix_pseudoinverse_transpose_inplace(P);
}

__global__ void kernel_null_double(int n, double *Ps, double *Cs)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n) return;
	double *P=Ps+12*idx;
	culaut::projection_matrix_source_position(P,Cs+idx*4);
}

__global__ void kernel_null_pinv_double(int n, double *Ps, double *Cs)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n) return;
	double *P=Ps+12*idx;
	culaut::projection_matrix_source_position(P,Cs+idx*4);
	culaut::projection_matrix_pseudoinverse_transpose_inplace(P);
}

//////////////////////////////////////////////////

void test_gpu_float(float *Ps, float *Pinv, float*Cs, int n)
{
	cudaDeviceSynchronize();
	cudaCheckState
	Utils::TimerWin32 time;
	time.startTimer();
	// Threads per block and problem size (one thread handles all parameters of one pair.)
	dim3 block_size;
	block_size.x=64;
	dim3 grid_size;
	grid_size.x = iDivUp(n,block_size.x);

	// Download input data
	CudaUtils::MemoryBlock<float> Ps_d(12*n,Ps);
	CudaUtils::MemoryBlock<float> Cs_d(4*n);
	cudaDeviceSynchronize();
	cudaCheckState
	std::cout << "download to device:\t" << time.getElapsedTime()*1000 << "ms\n";

	// Start kernel
	if (0)
		kernel_null_pinv_float<<<grid_size, block_size>>>(n, Ps_d, Cs_d); // slower.
	else
	{
		kernel_null_float<<<grid_size, block_size>>>(n, Ps_d, Cs_d);
		kernel_pinv_float<<<grid_size, block_size>>>(n, Ps_d);
	}
	cudaDeviceSynchronize();
	cudaCheckState
	std::cout << "computations:\t" << time.getElapsedTime()*1000 << "ms\n";

	//kernel_pinv_float
	//kernel_null_float
	//kernel_null_pinv_float
	//kernel_pinv_double
	//kernel_null_double
	//kernel_null_pinv_double

	Ps_d.readback(Pinv);
	Cs_d.readback(Cs);
	cudaDeviceSynchronize();
	cudaCheckState
	std::cout << "readback from device:\t" << time.getElapsedTime()*1000 << "ms\n";
	std::cout << "total GPU time spent:\t:" << time.getTotalTime()*1000 << "ms\n";
}

void test_gpu_double(double *Ps, double *Pinv, double*Cs, int n)
{
	cudaDeviceSynchronize();
	cudaCheckState
	Utils::TimerWin32 time;
	time.startTimer();
	// Threads per block and problem size (one thread handles all parameters of one pair.)
	dim3 block_size;
	block_size.x=64;
	dim3 grid_size;
	grid_size.x = iDivUp(n,block_size.x);

	// Download input data
	CudaUtils::MemoryBlock<double> Ps_d(12*n,Ps);
	CudaUtils::MemoryBlock<double> Cs_d(4*n);
	cudaDeviceSynchronize();
	cudaCheckState
	std::cout << "download to device:\t" << time.getElapsedTime()*1000 << "ms\n";

	// Start kernel
	if (0)
		kernel_null_pinv_double<<<grid_size, block_size>>>(n, Ps_d, Cs_d);
	else
	{
		kernel_null_double<<<grid_size, block_size>>>(n, Ps_d, Cs_d);
		kernel_pinv_double<<<grid_size, block_size>>>(n, Ps_d);
	}
	cudaDeviceSynchronize();
	cudaCheckState
	std::cout << "computations:\t" << time.getElapsedTime()*1000 << "ms\n";

	//kernel_pinv_double
	//kernel_null_double
	//kernel_null_pinv_double
	//kernel_pinv_double
	//kernel_null_double
	//kernel_null_pinv_double

	Ps_d.readback(Pinv);
	Cs_d.readback(Cs);
	cudaDeviceSynchronize();
	cudaCheckState
	std::cout << "readback from device:\t" << time.getElapsedTime()*1000 << "ms\n";
	std::cout << "total GPU time spent:\t:" << time.getTotalTime()*1000 << "ms\n";
}
