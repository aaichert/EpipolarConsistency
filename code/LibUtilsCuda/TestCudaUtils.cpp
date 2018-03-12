#include <LibProjectiveGeometry/SingularValueDecomposition.h>
#include <LibProjectiveGeometry/ProjectionMatrix.h>

#include <Utils/TimerWin32.hxx>

#include <random>
#include <iostream>

// Compile Cuda-Code as Host functions
#include <LibCudaUtils/culaut/culaut.hxx>
#include <LibCudaUtils/culaut/xgeinv.hxx>
#include <LibCudaUtils/culaut/xprojectionmatrix.hxx>


extern void test_gpu_float(float *Ps, float *Pinv, float*Cs, int n);
extern void test_gpu_double(double *Ps, double *Pinv, double*Cs, int n);


template <typename T>
void test_cpu(T *Ps, T *Pinvs, T *Cs, int n)
{
	Utils::TimerWin32 time;	
	time.startTimer();
	#pragma omp parallel for
	for (int idx=0;idx<n;idx++)
	{
		T *P=Ps+12*idx;
		T *PPinv=Pinvs+12*idx;
		T *C=Cs+4*idx;

		culaut::projection_matrix_source_position(P,C);
		culaut::xvcpy<T,T,12>(PPinv,P);
		culaut::projection_matrix_pseudoinverse_transpose_inplace(PPinv);
	}
	std::cout << "CPU total time: " << time.getElapsedTime()*1000 << "ms" << std::endl;
}

template <typename T>
std::pair<double,double> residuals(T* Ps, T *PinvTs, T* Cs, int n)
{
	// Compute errors (vector 2norm and Frobenius)
	double err_null=0, err_pinv=0;
	for (int i=0;i<n;i++)
	{
		Geometry::ProjectionMatrix P;
		culaut::xvcpy<double,T,12>(P.data(),Ps+12*i);
		Geometry::ProjectionMatrix PinvT;
		culaut::xvcpy<double,T,12>(PinvT.data(),PinvTs+12*i);
		Geometry::RP3Point C;
		culaut::xvcpy<double,T,4>(C.data(),Cs+4*i);

		err_null+=(P*C).norm();
		Eigen::Matrix3d id=P*PinvT.transpose();
		err_pinv+=(id-Eigen::Matrix3d::Identity()).norm();
	}
	return std::pair<double,double>(err_null/n,err_pinv/n);
}

int main()
{
	std::cout <<
		"Testing CUDA matrix inversion and null space computation.\n"
		"All errors are null/pinv euclidian/frobenius norms.\n";
	// Generate test data
	std::cout << "Generating test data...\n";
	int n=1024*1024;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-1,1);
	std::vector<Geometry::ProjectionMatrix> Ps;
	for (int i=0;i<n;i++)
	{
		Geometry::ProjectionMatrix P;
		for (int i=0;i<12;i++)
			P.data()[i]=distribution(generator);
		Ps.push_back(P);
	}

	// CPU Reference: Eigen
	std::cout << "Computing... (CPU Reference)\n";
	std::vector<Geometry::RP3Point> Cs(n);
	std::vector<Geometry::ProjectionMatrix> PinvTs(n);
	
	// pseudoinverse+null space at once.
	Utils::TimerWin32 time;
	time.startTimer();
	#pragma omp parallel for
	for (int i=0;i<n;i++)
	{
		Geometry::ProjectionMatrixInverse Pinv;
		Geometry::pseudoInverseAndNullspace(Ps[i],Pinv,Cs[i]);
		PinvTs[i]=Pinv.transpose();
	}
	std::cout << "CPU Eigen total time: " << (1000*time.getElapsedTime()) << "ms" << std::endl;
	auto residual_cpu=residuals(Ps[0].data(),PinvTs[0].data(),Cs[0].data(),n);
	std::cout << "CPU residuals: " <<  residual_cpu.first << " / " << residual_cpu.second << "\n###\n";

	// Copy data to plain arrays
	std::vector<float> Psf(n*12), PinvTsf(n*12), Csf(n*4);
	std::vector<double> Psd(n*12), PinvTsd(n*12), Csd(n*4);
	for (int i=0;i<n;i++)
		for (int j=0;j<12;j++)
			Psf[12*i+j]=(float)(Psd[12*i+j]=Ps[i].data()[j]);

	/// CPU float
	std::cout << "\n### CPU float\n";
	test_cpu(&Psf[0],&PinvTsf[0],&Csf[0],n);
	auto residual_cpu_float=residuals(&Psf[0],&PinvTsf[0],&Csf[0],n);
	std::cout << "CPU float residuals: " <<  residual_cpu_float.first << " / " << residual_cpu_float.second << "\n###\n";

	/// GPU float
	std::cout << "\n### GPU float\n";
	test_gpu_float(&Psf[0],&PinvTsf[0],&Csf[0],n);
	auto residual_gpu_float=residuals(&Psf[0],&PinvTsf[0],&Csf[0],n);
	std::cout << "GPU residuals: " <<  residual_gpu_float.first << " / " << residual_gpu_float.second << "\n###\n";

	/// GPU double
	std::cout << "\n### GPU double\n";
	test_gpu_double(&Psd[0],&PinvTsd[0],&Csd[0],n);
	auto residual_gpu_double=residuals(&Psd[0],&PinvTsd[0],&Csd[0],n);
	std::cout << "GPU residuals: " <<  residual_gpu_double.first << " / " << residual_gpu_double.second << "\n###\n";

	double cpu_gpu_error_null=0;
	double cpu_gpu_error_pinv=0;
	for (int i=0;i<n;i++)
	{
		for (int j=0;j<4;j++)
			cpu_gpu_error_null+=pow(Csf[i*4+j]-Cs[i].data()[j],2);
		for (int j=0;j<12;j++)
			cpu_gpu_error_pinv+=pow(PinvTsf[i*12+j]-PinvTs[i].data()[j],2);
	}
	std::cout << "GPU float CPU difference: " << cpu_gpu_error_null/n << " / " << cpu_gpu_error_pinv/n << "\n###\n";


	return 0;
}

