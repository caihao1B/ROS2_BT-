#include "acceptance_monitor.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char**argv){
 try {
  if(argc!=3)throw std::runtime_error("monitor INPUT.json OUTPUT.json");
  std::ifstream input(argv[1]);s17::json data;input>>data;
  auto result=s17::monitor(data);std::ofstream(argv[2])<<result.dump(2)<<'\n';
  return result.at("passed").get<bool>()?0:1;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
}
