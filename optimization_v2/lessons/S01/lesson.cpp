// Teaching equivalent, not a second production baseline validator.
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
namespace fs=std::filesystem;
int main(int argc,char**argv) {
  try {
    if(argc!=2)throw std::runtime_error("CONFIG_REQUIRED");
    std::ifstream file(argv[1]);nlohmann::json c;file>>c;
    if(c.at("ros_domain_id")!=119)throw std::runtime_error("DOMAIN_MISMATCH");
    auto root=fs::canonical(c.at("project_root").get<std::string>());
    for(const auto& item:c.at("required_files")) {
      auto p=fs::weakly_canonical(root/item.get<std::string>());
      auto relative=p.lexically_relative(root);
      if(relative.empty() || *relative.begin()=="..")throw std::runtime_error("PATH_OUTSIDE_PROJECT");
      if(!fs::is_regular_file(p))throw std::runtime_error("MISSING_RESOURCE: "+item.get<std::string>());
    }
    std::cout<<nlohmann::json({{"status","READY"},{"validated_files",c.at("required_files").size()}})<<'\n';
    return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
}
