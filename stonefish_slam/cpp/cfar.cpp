#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <Eigen/Dense>
#include <algorithm> 

/* CFAR이란?
 - cfar: constant false alarm rate
 - 노이즈가 많은 환경에서 목표를 자동으로 탐지하기 위한 방법
 - 방법: 주변환경의 평균 노이즈 수준에 맞춰, 동적으로 임계값을 조절하는 알고리즘.
 - 보호셀(guard cells):
    탐지를 시도하는 셀 바로 주변의 셀, 얘네들은 평균노이즈 계산에서 제외(이걸 보호셀로 하는 이유?: 만약 탐지 시도하는 셀이 진짜 타겟이 있는 셀이면 그 셀주변은 값이 높아질 수 있기 때문.)
 - 훈련셀(training cells):
    배경 노이즈 수준을 추정하기 위한 참고용 셀(보호셀을 피해서 양옆으로 조금 떨어진 셀들을 이용)
*/

typedef Eigen::MatrixXf MatrixXf;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;

// img: input image, train_hs(half-span): 훈련셀의 절반 수, guard_hs: 보호셀의 절반 수, tau: threshold 비율
// ca: cell averaging CFAR(훈련 셀 전체의 평균을 임계값으로 설정)
MatrixXb ca(const MatrixXf &img, int train_hs, int guard_hs, double tau) 
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float sum_train = 0;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if (std::abs(i - row) > guard_hs)
          sum_train += img(i, col);
      }
      ret(row, col) = img(row, col) > tau * sum_train / (2.0 * train_hs);
    }
  }
  return ret;
}

// soca: smallest of cell averaging(앞쪽 훈련, 뒤쪽 훈련 셀의 평균을 비교해서 더 작은 쪽을 임계값으로 설정)
MatrixXb soca(const MatrixXf &img, int train_hs, int guard_hs, double tau)
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float leading_sum = 0.0, lagging_sum = 0.0;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if ((i - row) > guard_hs)
          lagging_sum += img(i, col);
        else if ((i - row) < -guard_hs)
          leading_sum += img(i, col);
      }
      float sum_train = std::min(leading_sum, lagging_sum);
      ret(row, col) = img(row, col) > tau * sum_train / train_hs;
    }
  }
  return ret;
}

// goca: greatest of cell averaging(앞쪽 훈련, 뒤쪽 훈련 셀의 평균을 비교해서 더 작은 쪽을 큰값으로 설정)
MatrixXb goca(const MatrixXf &img, int train_hs, int guard_hs, double tau)
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float leading_sum = 0.0, lagging_sum = 0.0;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if ((i - row) > guard_hs)
          lagging_sum += img(i, col);
        else if ((i - row) < -guard_hs)
          leading_sum += img(i, col);
      }
      float sum_train = std::max(leading_sum, lagging_sum);
      ret(row, col) = img(row, col) > tau * sum_train / train_hs;
    }
  }
  return ret;
}

// os: order statistics(훈련셀을 정렬해서 k번째 값을 사용)
MatrixXb os(const MatrixXf &img, int train_hs, int guard_hs, int k, double tau)
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float leading_sum = 0.0, lagging_sum = 0.0;
      std::vector<float> train;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if (std::abs(i - row) > guard_hs)
          train.push_back(img(i, col));
      }
      std::nth_element(train.begin(), train.begin() + k, train.end());
      ret(row, col) = img(row, col) > tau * train[k];
    }
  }
  return ret;
}

std::pair<MatrixXb, MatrixXf> ca2(const MatrixXf &img, int train_hs, int guard_hs, double tau)
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());
  MatrixXf ret2 = MatrixXf::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float sum_train = 0;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if (std::abs(i - row) > guard_hs)
          sum_train += img(i, col);
      }
      ret(row, col) = img(row, col) > tau * sum_train / (2.0 * train_hs);
      ret2(row, col) = tau * sum_train / (2.0 * train_hs);
    }
  }
  return std::make_pair(ret, ret2);
}

std::pair<MatrixXb, MatrixXf> soca2(const MatrixXf &img, int train_hs, int guard_hs, double tau)
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());
  MatrixXf ret2 = MatrixXf::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float leading_sum = 0.0, lagging_sum = 0.0;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if ((i - row) > guard_hs)
          lagging_sum += img(i, col);
        else if ((i - row) < -guard_hs)
          leading_sum += img(i, col);
      }
      float sum_train = std::min(leading_sum, lagging_sum);
      ret(row, col) = img(row, col) > tau * sum_train / train_hs;
      ret2(row, col) = tau * sum_train / train_hs;
    }
  }
  return std::make_pair(ret, ret2);
}

std::pair<MatrixXb, MatrixXf> goca2(const MatrixXf &img, int train_hs, int guard_hs, double tau)
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());
  MatrixXf ret2 = MatrixXf::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float leading_sum = 0.0, lagging_sum = 0.0;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if ((i - row) > guard_hs)
          lagging_sum += img(i, col);
        else if ((i - row) < -guard_hs)
          leading_sum += img(i, col);
      }
      float sum_train = std::max(leading_sum, lagging_sum);
      ret(row, col) = img(row, col) > tau * sum_train / train_hs;
      ret2(row, col) = tau * sum_train / train_hs;
    }
  }
  return std::make_pair(ret, ret2);
}

std::pair<MatrixXb, MatrixXf> os2(const MatrixXf &img, int train_hs, int guard_hs, int k, double tau)
{
  MatrixXb ret = MatrixXb::Zero(img.rows(), img.cols());
  MatrixXf ret2 = MatrixXf::Zero(img.rows(), img.cols());

  for (int col = 0; col < img.cols(); ++col)
  {
    for (int row = train_hs + guard_hs; row < img.rows() - train_hs - guard_hs; ++row)
    {
      float leading_sum = 0.0, lagging_sum = 0.0;
      std::vector<float> train;
      for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i)
      {
        if (std::abs(i - row) > guard_hs)
          train.push_back(img(i, col));
      }
      std::nth_element(train.begin(), train.begin() + k, train.end());
      ret(row, col) = img(row, col) > tau * train[k];
      ret2(row, col) = tau * train[k];
    }
  }
  return std::make_pair(ret, ret2);
}

PYBIND11_MODULE(cfar, m)
{
  m.def("ca", &ca);
  m.def("soca", &soca);
  m.def("goca", &goca);
  m.def("os", &os);
  m.def("ca2", &ca2);
  m.def("soca2", &soca2);
  m.def("goca2", &goca2);
  m.def("os2", &os2);
}