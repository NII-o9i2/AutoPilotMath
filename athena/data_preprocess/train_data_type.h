//
// Created by SENSETIME\fengxiaotong on 24-8-7.
//
#pragma once
#include "cmath"
#include "iostream"
#include "vector"

namespace DLP{


inline float normalize_angle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return static_cast<float>(angle);
};

inline float angle_between_2d_vectors(double ctr_x, double ctr_y, double nbr_x,
                               double nbr_y) {
  double cross_product = ctr_x * nbr_y - ctr_y * nbr_x;
  double dot_product = ctr_x * nbr_x + ctr_y * nbr_y;
  double angle = std::atan2(cross_product, dot_product);

  return static_cast<float>(normalize_angle(angle));
};

class TensorInfo{
 public:
  TensorInfo()=default;
  TensorInfo(std::vector<size_t> &_shape){
    shape_.clear();
    shape_ = _shape;
  };
  ~TensorInfo()=default;

  // Getter for shape
  const std::vector<size_t>& shape() const {
    return shape_;
  }

  // Setter for shape
  void set_shape(const std::vector<size_t>& shape) {
    shape_ = shape;
  }

  void reset_shape(){
    shape_.clear();
  }

  // Friend function to overload the << operator for adding integers to shape_
  friend TensorInfo& operator<<(TensorInfo& tensor_info, size_t value) {
    tensor_info.shape_.emplace_back(value);
    return tensor_info;
  }

 private:
  std::vector<size_t> shape_;
};

struct TrainDataSet{
  int id;
  int time_;
};


class Tensor{
 public:
  Tensor()= default;

  const std::vector<size_t>& shape() const {
    return info_.shape();
  }

 protected:
  TensorInfo info_;
};

template<typename T>
class TensorD1 : public Tensor{
 public:
  TensorD1() = default;
  explicit TensorD1(size_t shape){
    info_.reset_shape();
    info_<<shape;
    data_.clear();
    data_.resize(shape,static_cast<T>(0));
  }

  void create(size_t shape){
    info_.reset_shape();
    info_<<shape;
    data_.clear();
    data_.resize(shape,static_cast<T>(0));
  }

  std::vector<T> &data(){
    return data_;
  }
  const std::vector<T> &value() const{
    return data_;
  }

  void cover(int i_cover, TensorD1<T>& new_data){
    for (int i = 0; i < new_data.shape()[0]; i++){
      if (i + i_cover >= info_.shape()[0]){
        break;
      }
      data_.data()[i+i_cover] = new_data.data()[i];
    }
  }

  void continuous_data(std::vector<T>& vec){
    vec.clear();
    for (const auto& value : data_){
      vec.emplace_back(value);
    }
  }
 private:
  std::vector<T> data_;
};

template<typename T>
class TensorD2: public Tensor {
 public:
  TensorD2() = default;
  TensorD2(size_t row, size_t col){
    info_.reset_shape();
    info_<< row << col;
    data_.clear();
    data_.resize(row);
    for (int i =0 ; i < row; i++){
      data_[i].resize(col,static_cast<T>(0));
    }
  }

  void create(size_t row, size_t col){
    info_.reset_shape();
    info_<< row << col;
    data_.clear();
    data_.resize(row);
    for (int i =0 ; i < row; i++){
      data_[i].resize(col,static_cast<T>(0));
    }
  }

  std::vector<std::vector<T>> &data(){
    return data_;
  }

  const std::vector<std::vector<T>> &value() const{
    return data_;
  }

  bool cat(TensorD2<T>& new_data, int dim = 0){
    if (dim > 1){
      return false;
    }
    if (dim == 0 && new_data.shape()[1] != info_.shape()[1]){
      return false;
    }
    if (dim == 1 && new_data.shape()[0] != info_.shape()[0]){
      return false;
    }

    if (dim == 0){
      info_.set_shape({info_.shape()[0]+new_data.shape()[0],info_.shape()[1]});
      for(int i = 0; i < new_data.shape()[0];i++){
        data_.emplace_back(new_data.data()[i]);
      }
    }
    if (dim == 1){
      info_.set_shape({info_.shape()[0],info_.shape()[1]+new_data.shape()[1]});
      for(int i = 0; i < info_.shape()[0];i++){
        for(int j = 0; j < new_data.shape()[1];j++){
          data_[i].emplace_back(new_data.data()[i][j]);
        }
      }
    }
    return true;
  }

  void cover(int i_cover, int j_cover, TensorD2<T>& new_data){
    for (int i = 0; i < new_data.shape()[0]; i++){
      if (i + i_cover >= info_.shape()[0]){
        break;
      }
      for (int j = 0; j < new_data.shape()[1]; j++){
        if (j + j_cover >= info_.shape()[1]){
          continue;
        }
        data_.data()[i+i_cover][j+j_cover] = new_data.data()[i][j];
      }
    }
  }

  void continuous_data(std::vector<T>& vec){
    vec.clear();
    for (const auto& row : data_) {
      vec.insert(vec.end(), row.begin(), row.end());
    }
  }
 private:
  std::vector<std::vector<T>> data_;
};

template<typename T>
class TensorD3: public Tensor {
 public:
  TensorD3() = default;
  TensorD3(size_t row, size_t col, size_t dim){
    info_.reset_shape();
    info_<< row << col << dim;
    data_.clear();
    data_.resize(row);
    for (int i =0 ; i < row; i++){
      data_[i].resize(col);
      for(int j = 0; j < col; j++){
        data_[i][j].resize(dim, static_cast<T>(0));
      }
    }
  }

  void create(size_t row, size_t col, size_t dim){
    info_.reset_shape();
    info_<< row << col << dim;
    data_.clear();
    data_.resize(row);
    for (int i =0 ; i < row; i++){
      data_[i].resize(col);
      for(int j = 0; j < col; j++){
        data_[i][j].resize(dim, static_cast<T>(0));
      }
    }
  }
  std::vector<std::vector<std::vector<T>>> &data(){
    return data_;
  }

  void continuous_data(std::vector<T>& vec){
    vec.clear();
    for (const auto& row : data_) {
      for (const auto& col : row){
        vec.insert(vec.end(), col.begin(), col.end());
      }
    }
  }
 private:
  std::vector<std::vector<std::vector<T>>> data_;
};

template <typename T>
TensorD1<T> zeros(size_t size){
  return TensorD1<T>(size);
}

template <typename T>
TensorD2<T> zeros(size_t row, size_t col){
  return TensorD2<T>(row,col);
}

template <typename T>
TensorD3<T> zeros(size_t row, size_t col, size_t dim){
  return TensorD3<T>(row,col,dim);
}


}
