#include <device_launch_parameters.h>
#include <cuda_runtime.h>
#include <cuda.h>
#include <curand_kernel.h>
#include <pcl/gpu/containers/kernel_containers.h>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/features/features.hpp>
#include "zeta-ransac.h"


struct RansacEstimator
{   	
    enum
    {
        //CTA_SIZE = 512,
        //WAPRS = 1 << 5,
        //MIN_NEIGHBOORS = 1
    };

	public:
		unsigned int max_skip;
		unsigned int sample_size;
		int indices_size;
		int* gpuTargetIndices;
		int* gpuSourceIndices;
		int* shuffled_indices;
		pcl::PointXYZ* gpuTargetCloud;
		pcl::PointXYZ* gpuSourceCloud;
		float inlier_threshold;
		unsigned int model_size;
		int* result_inliers;
		Eigen::Matrix<float, 16, 1>* result_modelCoeffs;
		float considerasZero;
		float precision;
	
	RansacEstimator(int indices_size_input, float inlier_threshold_input, unsigned int model_size_input, unsigned int sample_size_input, unsigned int max_skip_input, 
		pcl::PointXYZ* gpuTargetCloud_input, pcl::PointXYZ* gpuSourceCloud_input, int* gpuTargetIndices_input, int* gpuSourceIndices_input, int* result_inliers_input, 
		Eigen::Matrix<float, 16, 1>* result_modelCoeffs_input, int* shuffled_indices_input, float considerasZero_input, float precision_input)
	{
		indices_size = indices_size_input;
		inlier_threshold = inlier_threshold_input;
		model_size = model_size_input;
		sample_size = sample_size_input;
		max_skip = max_skip_input;
		gpuTargetCloud = gpuTargetCloud_input;
		gpuSourceCloud = gpuSourceCloud_input;
		gpuTargetIndices = gpuTargetIndices_input;
		gpuSourceIndices = gpuSourceIndices_input;
		result_inliers = result_inliers_input;
		result_modelCoeffs = result_modelCoeffs_input;
		shuffled_indices = shuffled_indices_input;
		considerasZero = considerasZero_input;
		precision = precision_input;
	}
	
	__device__ float determinant(Eigen::Matrix<float, 3, 3> matrix){

		float a = matrix(0, 0);
		float b = matrix(0, 1);
		float c = matrix(0, 2);
		float d = matrix(1, 0);
		float e = matrix(1, 1);
		float f = matrix(1, 2);
		float g = matrix(2, 0);
		float h = matrix(2, 1);
		float i = matrix(2, 2);

		return(a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g));
	}
	
	__device__ void static multiplyJacobi(float first_m_c, float first_m_s, float second_m_c, float second_m_s, float& product_m_c, float& product_m_s)
	{
		product_m_c = first_m_c * second_m_c - Eigen::numext::conj(first_m_s) * second_m_s;
	    product_m_s = Eigen::numext::conj( first_m_c * Eigen::numext::conj(second_m_s) + 
										   Eigen::numext::conj(first_m_s) * Eigen::numext::conj(second_m_c)
										  );
		return;
	}
		
	__device__ bool static makeJacobi(float &m_c, float &m_s, const float x, const float y, const float z, float considerasZero)
	{
		float deno = float(2) * fabsf(y);
		if (deno < considerasZero)
		{
			m_c = float(1);
			m_s = float(0);
			return false;
		}
		else
		{
			float tau = (x - z) / deno;
			float w = sqrtf(Eigen::numext::abs2(tau) + float(1));
			float t;
			if (tau > float(0))
			{
				t = float(1) / (tau + w);
			}
			else
			{
				t = float(1) / (tau - w);
			}
			float sign_t = t > float(0) ? float(1) : float(-1);
			float n = float(1) / sqrtf(Eigen::numext::abs2(t) + float(1));
			m_s = -sign_t * (Eigen::numext::conj(y) / fabsf(y)) * fabsf(t) * n;
			m_c = n;
			return true;
		}
	}

	__device__ static void apply_rotation_in_the_plane_left(Eigen::Matrix<float, 1, 3> &row, Eigen::Matrix<float, 1, 3> &col, float m_c, float m_s) {
		int size = 3; 
		if (m_c == float(1) && m_s == float(0)) { return; }	
		for (int i = 0; i < size; ++i)
		{
			float xi = row(0,i);
			float yi = col(0,i);
			row(0, i) = m_c * xi + Eigen::numext::conj(m_s) * yi;
			col(0, i) = -m_s * xi + Eigen::numext::conj(m_c) * yi;

		}
	}

	__device__ static void apply_rotation_in_the_plane_right(Eigen::Matrix<float, 3, 1> &row, Eigen::Matrix<float, 3, 1> &col, float m_c, float m_s) {
		int size = 3;
		if (m_c == float(1) && m_s == float(0)) { return; }
		for (int i = 0; i < size; ++i)
		{
			float xi = row(i, 0);
			float yi = col(i, 0);
			row(i, 0) = m_c * xi + Eigen::numext::conj(m_s) * yi;
			col(i, 0) = -m_s * xi + Eigen::numext::conj(m_c) * yi;
		}
	}

	__device__ static void applyOnTheLeft(Eigen::Matrix<float, 3, 3> *matrix, float m_c, float m_s, int p, int q) {

		Eigen::Matrix<float, 1, 3> x = matrix->row(p);
		Eigen::Matrix<float, 1, 3> y = matrix->row(q);
		apply_rotation_in_the_plane_left(x, y, m_c, m_s);
	    matrix->row(p) = x;
	    matrix->row(q) = y;
	}

	__device__ static void applyOnTheRight(Eigen::Matrix<float, 3, 3> *matrix, float m_c, float m_s, int p, int q) {

		Eigen::Matrix<float, 3, 1> x = matrix->col(p);
		Eigen::Matrix<float, 3, 1> y = matrix->col(q);
		apply_rotation_in_the_plane_right(x, y, m_c, -1 * Eigen::numext::conj(m_s));
	    matrix->col(p) = x;
		matrix->col(q) = y;
	}

	__device__ static void apply_rotation_in_the_plane_left_2x2(Eigen::Matrix<float, 1, 2> &row, Eigen::Matrix<float, 1, 2> &col , float m_c, float m_s) {

		int size = 2;
		if (m_c == float(1) && m_s == float(0)) { return; }
		for (int i = 0; i < size; ++i)
		{
			float xi = row(0,i);
			float yi = col(0,i);
			row(0,i) = m_c * xi + Eigen::numext::conj(m_s) * yi;
			col(0,i) = -m_s * xi + Eigen::numext::conj(m_c) * yi;
		}
	}		
	
	__device__ static void apply_rotation_in_the_plane_right_2x2(Eigen::Matrix<float, 2, 1> &row, Eigen::Matrix<float, 2, 1> &col , float m_c, float m_s) {

		int size = 2;
		if (m_c == float(1) && m_s == float(0)) { return; }
		for (int i = 0; i < size; ++i)
		{
			float xi = row(i,0);
			float yi = col(i,0);
			row(i,0) = m_c * xi + Eigen::numext::conj(m_s) * yi;
			col(i,0) = -m_s * xi + Eigen::numext::conj(m_c) * yi;
		}
	}	
	

	__device__ __forceinline__ void real_2x2_jacobi_svd(Eigen::Matrix<float, 3, 3>* m_workMatrix, int p, int q, 
														float& left_m_c, float& left_m_s, float& right_m_c, float& right_m_s, float considerasZero, int idx)
	{
		Eigen::Matrix<float, 2, 2> mat;
		mat << Eigen::numext::real(m_workMatrix->coeff(p, p)), Eigen::numext::real(m_workMatrix->coeff(p, q)),
			Eigen::numext::real(m_workMatrix->coeff(q, p)), Eigen::numext::real(m_workMatrix->coeff(q, q));
		float m_c, m_s;
		float t = mat.coeff(0, 0) + mat.coeff(1, 1);
		float d = mat.coeff(1, 0) - mat.coeff(0, 1);

		if (fabsf(d) < considerasZero)
		{
			m_s = 0.0;
			m_c = 1.0;
		}
		else
		{
			float u = t / d;
			float tmp = sqrtf(1.0 + Eigen::numext::abs2(u));
			m_s = 1.0 / tmp;
			m_c = u / tmp;
		}

		Eigen::Matrix<float, 1, 2> row_input = mat.row(0);
		Eigen::Matrix<float, 1, 2> col_input  = mat.row(1);
		apply_rotation_in_the_plane_left_2x2(row_input, col_input, m_c, m_s);
		mat.row(0) = row_input;
		mat.row(1) = col_input;
		makeJacobi(right_m_c, right_m_s, Eigen::numext::real(mat.coeff(0, 0)), mat.coeff(0, 1), Eigen::numext::real(mat.coeff(1, 1)), considerasZero);
		
		Eigen::Matrix<float, 1, 2> testrow_input = mat.row(0);
		Eigen::Matrix<float, 1, 2> testcol_input  = mat.row(1);
		apply_rotation_in_the_plane_left_2x2(testrow_input, testcol_input,  Eigen::numext::conj(m_c), -m_s);
		mat.row(0) = row_input;
		mat.row(1) = col_input; 
		
		Eigen::Matrix<float, 2, 1> test2row_input = mat.col(0);
		Eigen::Matrix<float, 2, 1> test2col_input  = mat.col(1);
		apply_rotation_in_the_plane_right_2x2(test2row_input, test2col_input, m_c, m_s);
		mat.row(0) = row_input;
		mat.row(1) = col_input;
		 
		multiplyJacobi(m_c, m_s, right_m_c, -1 * Eigen::numext::conj(right_m_s), left_m_c, left_m_s);
			
	}


	__device__ __forceinline__ void jacobiSVD(Eigen::Matrix<float, 3, 3> &m_matrixU, Eigen::Matrix<float, 3, 3> &m_matrixV, Eigen::Matrix<float, 3, 3> &m_workMatrix, float considerAsZero, int idx, int m_cols, int m_diagSize, int m_rows, 
											  Eigen::Matrix<float, 3, 1> &m_singularValues, Eigen::Matrix<float, 3, 3> &sigma){

		// step 1. The R-SVD step: we use a QR decomposition to reduce to the case of a square matrix 
		m_workMatrix = (sigma.block<3, 3>(0, 0));
		m_matrixU.setIdentity(m_rows, m_rows);
		m_matrixV.setIdentity(m_cols, m_cols);

		// step 2. The main Jacobi SVD iteration. 
		float maxDiagEntry = m_workMatrix.cwiseAbs().diagonal().maxCoeff();

		bool finished = false;
		Eigen::Matrix<float, 3, 1> S1;
		Eigen::Matrix<float, 3, 3> s_diag1;

		while (!finished)
		{
			finished = true;
			for (int p = 1; p < m_diagSize; ++p)
			{
				for (int q = 0; q < p; ++q)
				{
					// if this 2x2 sub-matrix is not diagonal already...
					// notice that this comparison will evaluate to false if any NaN is involved, ensuring that NaN's don't
					// keep us iterating forever. Similarly, small denormal numbers are considered zero.
					float threshold = Eigen::numext::maxi<float>(considerAsZero, precision * maxDiagEntry);						
					if(fabsf(m_workMatrix.coeff(p, q)) > threshold || fabsf(m_workMatrix.coeff(q, p)) >  threshold)
					{
						finished = false;		
						float left_m_c, left_m_s, right_m_c, right_m_s;
						real_2x2_jacobi_svd(&m_workMatrix, p, q, left_m_c, left_m_s, right_m_c, right_m_s, considerAsZero, idx);
						
						applyOnTheLeft(&m_workMatrix, left_m_c, left_m_s, p, q);
						applyOnTheRight(&m_matrixU, left_m_c, -Eigen::numext::conj(left_m_s), p, q);
						
						applyOnTheRight(&m_workMatrix, right_m_c, right_m_s, p, q);
						applyOnTheRight(&m_matrixV, right_m_c, right_m_s, p, q);
						maxDiagEntry = Eigen::numext::maxi<float>(maxDiagEntry, Eigen::numext::maxi<float>(fabsf(m_workMatrix.coeff(p, p)), fabsf(m_workMatrix.coeff(q, q))));
					}
				}
			}
		} 


		// step 3. The work matrix is now diagonal, so ensure it's positive so its diagonal entries are the singular values 
		for (int i = 0; i < m_diagSize; ++i)
		{
			// For a complex matrix, some diagonal coefficients might note have been
			// treated by svd_precondition_2x2_block_to_be_real, and the imaginary part
			// of some diagonal entry might not be null.
			if (Eigen::NumTraits<float>::IsComplex && fabsf(Eigen::numext::imag(m_workMatrix.coeff(i, i))) > considerAsZero)
			{
				float a = fabsf(m_workMatrix.coeff(i, i));
				m_singularValues.coeffRef(i) = fabsf(a);
				m_matrixU.col(i) *= m_workMatrix.coeff(i, i) / a;
			}
			else
			{		
				// m_workMatrix.coeff(i,i) is already real, no difficulty:
				float a = Eigen::numext::real(m_workMatrix.coeff(i, i));
				m_singularValues.coeffRef(i) = fabsf(a);
				if ((a < float(0))){
					m_matrixU.col(i) = -m_matrixU.col(i);
				} 
			}
		}
		

		//m_singularValues *= scale;	
		// step 4. Sort singular values in descending order and compute the number of nonzero singular values
		float m_nonzeroSingularValues = m_diagSize;
		for (int i = 0; i < m_diagSize; i++)
		{
			
			int pos;
			float maxRemainingSingularValue = m_singularValues.tail(m_diagSize - i).maxCoeff(&pos);
			if (maxRemainingSingularValue == float(0))
			{
				//if (idx == 0){printf("triggered 2.5 \n");}
				m_nonzeroSingularValues = i;
				break;
			}
			if (pos)
			{
				//if (idx == 0){printf("triggered 3 \n");}
				pos += i;
				//std::swap(m_singularValues.coeffRef(i), m_singularValues.coeffRef(pos));
				float temp = m_singularValues.coeff(i);
				m_singularValues.coeffRef(i) = m_singularValues.coeffRef(pos);
				m_singularValues.coeffRef(pos) = temp;
				m_matrixU.col(pos).swap(m_matrixU.col(i));
				m_matrixV.col(pos).swap(m_matrixV.col(i));
			}
		}
	}

			
	__device__ __forceinline__ void operator () ()
	{
		int idx = threadIdx.x + blockIdx.x * blockDim.x;
		int tid = threadIdx.x;

		if (idx > this->max_skip) { return; }

		//getSamples
		int thread_targets[3];
		int thread_samples[3];
		
		for (int i = 0; i < 3; ++i)
		{
			// repeat selections bug  is still present
			//get random numbers
			curandState state;
			long long int something = clock64() + idx;
			curand_init(something, idx, 0, &state);
			int random_index = trunc(curand_uniform_double(&state) * (this->indices_size));
			//int temp = this->shuffled_indices[i];
			//this->shuffled_indices[i] = this->shuffled_indices[i + random_index];
			//this->shuffled_indices[i + random_index] = temp;
			thread_samples[i] = this->gpuSourceIndices[random_index];
			thread_targets[i] = this->gpuTargetIndices[random_index];
		}

		//computeModelCefficients
		Eigen::Matrix<float, 3, Eigen::Dynamic> src(3, 3);
		Eigen::Matrix<float, 3, Eigen::Dynamic> tgt(3, 3);

		for (int i = 0; i < 3; ++i)
		{
			src(0, i) = static_cast<pcl::PointXYZ>(this->gpuSourceCloud[thread_samples[i]]).x;
			src(1, i) = static_cast<pcl::PointXYZ>(this->gpuSourceCloud[thread_samples[i]]).y;
			src(2, i) = static_cast<pcl::PointXYZ>(this->gpuSourceCloud[thread_samples[i]]).z;

			tgt(0, i) = static_cast<pcl::PointXYZ>(this->gpuTargetCloud[thread_targets[i]]).x;
			tgt(1, i) = static_cast<pcl::PointXYZ>(this->gpuTargetCloud[thread_targets[i]]).y;
			tgt(2, i) = static_cast<pcl::PointXYZ>(this->gpuTargetCloud[thread_targets[i]]).z;
		}


		//Eigen::Matrix4d transformation_matrix = Eigen::umeyama(src, tgt, false);
		bool with_scaling = false;
		const Eigen::Index m = 3; // dimension
		const Eigen::Index n = 3; // number of measurements

		// required for demeaning ...
		const float one_over_n = 1.0 / n;
		// computation of mean
		const Eigen::Matrix<float, 3, 1> src_mean = src.rowwise().sum() * one_over_n;
		const Eigen::Matrix<float, 3, 1> tgt_mean = tgt.rowwise().sum() * one_over_n;

		// demeaning of src and dst points
		const Eigen::Matrix<float, 3, 3> tgt_demean = tgt.colwise() - tgt_mean;
		const Eigen::Matrix<float, 3, 3> src_demean = src.colwise() - src_mean;

		// Eq. (36)-(37)
		const float src_var = src_demean.rowwise().squaredNorm().sum() * one_over_n;
	
		// Eq. (38)
		const Eigen::Matrix<float, 3, 3> sigma = one_over_n * tgt_demean * src_demean.transpose();
		
		Eigen::Matrix<float, 3, 3> m_matrixU;
		Eigen::Matrix<float, 3, 3> m_matrixV;
		Eigen::Index m_rows = 3;
		Eigen::Index m_cols = 3;
		const int m_diagSize = 3; //equal to the number of rows
		Eigen::Matrix<float, 3, 1> m_singularValues;
		Eigen::Matrix<float, 3, 3> m_workMatrix;

		// currently we stop when we reach precision 2*epsilon as the last bit of precision can require an unreasonable number of iterations,
		// only worsening the precision of U and V as we accumulate more rotations
		const float precision = this->precision;

		// limit for denormal numbers to be considered zero in order to avoid infinite loops (see bug 286)
		const float considerAsZero = this->considerasZero;
		
		// Scaling factor to reduce over/under-flows
		//float scale = sigma.cwiseAbs().maxCoeff();
		//if (scale == float(0)) scale = float(1);		
		//float one_over_scale = 1.0 / scale;
		
		// step 1. The R-SVD step: we use a QR decomposition to reduce to the case of a square matrix 
		m_workMatrix = (sigma.block<3, 3>(0, 0));
		m_matrixU.setIdentity(m_rows, m_rows);
		m_matrixV.setIdentity(m_cols, m_cols);

		// step 2. The main Jacobi SVD iteration. 
		float maxDiagEntry = m_workMatrix.cwiseAbs().diagonal().maxCoeff();

		bool finished = false;
		Eigen::Matrix<float, 3, 1> S1;
		Eigen::Matrix<float, 3, 3> s_diag1;

		while (!finished)
		{
			finished = true;
			for (int p = 1; p < m_diagSize; ++p)
			{
				for (int q = 0; q < p; ++q)
				{
					// if this 2x2 sub-matrix is not diagonal already...
					// notice that this comparison will evaluate to false if any NaN is involved, ensuring that NaN's don't
					// keep us iterating forever. Similarly, small denormal numbers are considered zero.
					float threshold = Eigen::numext::maxi<float>(considerAsZero, precision * maxDiagEntry);						
					if(fabsf(m_workMatrix.coeff(p, q)) > threshold || fabsf(m_workMatrix.coeff(q, p)) >  threshold)
					{
						finished = false;		
						float left_m_c, left_m_s, right_m_c, right_m_s;
						real_2x2_jacobi_svd(&m_workMatrix, p, q, left_m_c, left_m_s, right_m_c, right_m_s, considerAsZero, idx);
						
						applyOnTheLeft(&m_workMatrix, left_m_c, left_m_s, p, q);
						applyOnTheRight(&m_matrixU, left_m_c, -Eigen::numext::conj(left_m_s), p, q);
						
						applyOnTheRight(&m_workMatrix, right_m_c, right_m_s, p, q);
						applyOnTheRight(&m_matrixV, right_m_c, right_m_s, p, q);
						maxDiagEntry = Eigen::numext::maxi<float>(maxDiagEntry, Eigen::numext::maxi<float>(fabsf(m_workMatrix.coeff(p, p)), fabsf(m_workMatrix.coeff(q, q))));
					}
				}
			}
		} 


		// step 3. The work matrix is now diagonal, so ensure it's positive so its diagonal entries are the singular values 
		for (int i = 0; i < m_diagSize; ++i)
		{
			// For a complex matrix, some diagonal coefficients might note have been
			// treated by svd_precondition_2x2_block_to_be_real, and the imaginary part
			// of some diagonal entry might not be null.
			if (Eigen::NumTraits<float>::IsComplex && fabsf(Eigen::numext::imag(m_workMatrix.coeff(i, i))) > considerAsZero)
			{
				float a = fabsf(m_workMatrix.coeff(i, i));
				m_singularValues.coeffRef(i) = fabsf(a);
				m_matrixU.col(i) *= m_workMatrix.coeff(i, i) / a;
			}
			else
			{		
				float a = Eigen::numext::real(m_workMatrix.coeff(i, i));
				m_singularValues.coeffRef(i) = fabsf(a);
				if ((a < float(0))){
					m_matrixU.col(i) = -m_matrixU.col(i);
				} 
			}
		}
		


		// step 4. Sort singular values in descending order and compute the number of nonzero singular values
		float m_nonzeroSingularValues = m_diagSize;
		for (int i = 0; i < m_diagSize; i++)
		{
			
			int pos;
			float maxRemainingSingularValue = m_singularValues.tail(m_diagSize - i).maxCoeff(&pos);
			if (maxRemainingSingularValue == float(0))
			{
				m_nonzeroSingularValues = i;
				break;
			}
			if (pos)
			{
				pos += i;
				float temp = m_singularValues.coeff(i);
				m_singularValues.coeffRef(i) = m_singularValues.coeffRef(pos);
				m_singularValues.coeffRef(pos) = temp;
				m_matrixU.col(pos).swap(m_matrixU.col(i));
				m_matrixV.col(pos).swap(m_matrixV.col(i));
			}
		}

		// Initialize the resulting transformation with an identity matrix...
		Eigen::Matrix<float, 4, 4> transformation_matrix = Eigen::Matrix<float, 4, 4>::Identity(m + 1, m + 1);

		// Eq. (39)
		Eigen::Matrix<float, 3, 1> S = Eigen::Matrix<float, 3, 1>::Ones(m);
		if ( determinant(m_matrixU) * determinant(m_matrixV) < 0) S(m - 1) = -1; 

		// Eq. (40) and (43)
		Eigen::Matrix<float, 3, 3> s_diag = S.asDiagonal();
		transformation_matrix.block<3, 3>(0, 0) = m_matrixU * s_diag * m_matrixV.transpose();	
		transformation_matrix.col(m).head(m) = tgt_mean;
		transformation_matrix.col(m).head(m) -= transformation_matrix.topLeftCorner(m, m) * src_mean;
		//end umeyama

	
		// Return the correct transformation
		Eigen::Matrix<float, 16, 1> model_coeffs;
		model_coeffs.segment<4>(0).matrix() = transformation_matrix.cast<float>().row(0);
		model_coeffs.segment<4>(4).matrix() = transformation_matrix.cast<float>().row(1);
		model_coeffs.segment<4>(8).matrix() = transformation_matrix.cast<float>().row(2);
		model_coeffs.segment<4>(12).matrix() = transformation_matrix.cast<float>().row(3);

		int nr_p = 0;
		for (size_t i = 0; i < this->indices_size; ++i)
		{
			
			Eigen::Vector4f pt_src(static_cast<pcl::PointXYZ>(this->gpuSourceCloud[this->gpuSourceIndices[i]]).x,
								   static_cast<pcl::PointXYZ>(this->gpuSourceCloud[this->gpuSourceIndices[i]]).y,
								   static_cast<pcl::PointXYZ>(this->gpuSourceCloud[this->gpuSourceIndices[i]]).z,
								   1);
			Eigen::Vector4f pt_tgt(static_cast<pcl::PointXYZ>(this->gpuTargetCloud[this->gpuTargetIndices[i]]).x,
								   static_cast<pcl::PointXYZ>(this->gpuTargetCloud[this->gpuTargetIndices[i]]).y,
								   static_cast<pcl::PointXYZ>(this->gpuTargetCloud[this->gpuTargetIndices[i]]).z,
								   1);
			
			Eigen::Vector4f p_tr(transformation_matrix * pt_src);
			float squared_dist = (p_tr - pt_tgt).squaredNorm();
			if (squared_dist < this->inlier_threshold * this->inlier_threshold) { nr_p++; }
		}
		this->result_inliers[idx] = nr_p;
		this->result_modelCoeffs[idx] = model_coeffs;
		return;
	}

};


__global__ void EstimateNormaslKernel(RansacEstimator est) { est(); }


__global__ void MyKernel(int* array, int arrayCount)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx < arrayCount) {
		array[idx] *= array[idx];
	}
}
void computeRansac(int indices_size, unsigned int sample_size, const unsigned max_skip, pcl::PointXYZ* TargetCloud,
	pcl::PointXYZ* SourceCloud, int* TargetIndices, int* SourceIndices, float inlier_threshold, unsigned int model_size,
	int* Inliers, Eigen::Matrix<float, 16, 1>* Coeffs, int* shuffledIndices,float considerasZero, float precision
)
{
	if (indices_size < sample_size) { return; }

	RansacEstimator est(indices_size, inlier_threshold, model_size, sample_size, max_skip, TargetCloud, SourceCloud,
						TargetIndices, SourceIndices, Inliers, Coeffs, shuffledIndices,considerasZero, precision);


	int blockSize; 
	int minGridSize;
	int gridSize;
	

	blockSize = 32;
	gridSize = (max_skip + blockSize - 1)/blockSize;


	EstimateNormaslKernel << <gridSize, blockSize >> > (est);

	cudaSafeCall(cudaGetLastError());
	cudaSafeCall(cudaDeviceSynchronize());

	/*
	int maxAcitveBlocks;
	cudaOccupancyMaxActiveBlocksPerMultiprocessor(&maxAcitveBlocks, EstimateNormaslKernel, blockSize, 0);
	
	int device; 
	cudaDeviceProp props; 
	cudaGetDevice(&device);
	cudaGetDeviceProperties(&props, device);

	float occupancy = (float)(maxAcitveBlocks * blockSize / props.warpSize) / 
					 (float)(props.maxThreadsPerMultiProcessor / props.warpSize);
	printf("Theoretical occupancy %f, max threads per processor %i, warp size : %i \n", occupancy, props.maxThreadsPerMultiProcessor,props.warpSize);
	*/
}