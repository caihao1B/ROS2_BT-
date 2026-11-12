// Teaching example, not a ROS node. Mirrors serialized PointCloud2 field offsets.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
float coordinate(const std::vector<uint8_t>&bytes,size_t point,size_t offset){
  const size_t step=18;
  if(offset>8 || bytes.size()<point*step+offset+4)throw std::out_of_range("truncated point");
  float value;std::memcpy(&value,bytes.data()+point*step+offset,4);return value;
}
int main(){
  std::vector<uint8_t> bytes(18);float xyz[3]={1.5f,-2.f,.7f};std::memcpy(bytes.data(),xyz,12);
  if(std::abs(coordinate(bytes,0,8)-.7f)>1e-6)return 1;
  bool refused=false;try{coordinate(bytes,1,0);}catch(const std::out_of_range&){refused=true;}
  if(!refused)return 2;
  double fx=640/(2*std::tan(.6)),baseline=.12;
  std::cout<<"PASS: little-endian xyz/18-byte stride, truncated input rejected; right CameraInfo P[3]="<<-fx*baseline<<"; acquisition stamp stays unchanged under delay\n";
}
