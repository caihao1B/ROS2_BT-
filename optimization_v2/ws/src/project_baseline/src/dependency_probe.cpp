#include <fields2cover.h>
#include <metis.h>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

// Compile/link the actual installed v2 API. This is not a Nav2 coverage-server test.
int main(int argc, char **argv) {
  try {
    const double width = argc > 1 ? std::stod(argv[1]) : 2.0;
    if (!std::isfinite(width) || width <= 0) throw std::invalid_argument("INVALID_TOOL_WIDTH");
    F2CLinearRing ring({F2CPoint(0,0), F2CPoint(20,0), F2CPoint(20,12),
                       F2CPoint(0,12), F2CPoint(0,0)});
    F2CCell cell(ring);
    f2c::hg::ConstHL headlands;
    auto mainland = headlands.generateHeadlands(F2CCells(cell), 1.0);
    f2c::sg::BruteForce generator;
    auto swaths = generator.generateSwaths(0.0, width, mainland.getGeometry(0));
    f2c::rp::BoustrophedonOrder order;
    auto sorted = order.genSortedSwaths(swaths);
    double length=0.0;
    for (const auto &swath : sorted) length += swath.length();
    if (sorted.size()==0 || length<=0) throw std::runtime_error("NO_SWATHS");
    // Six nodes in a path; three nonempty candidate parts. Full geometry is S11.
    idx_t nv=6, ncon=1, nparts=3, objective=0;
    idx_t xadj[]={0,1,3,5,7,9,10};
    idx_t adj[]={1,0,2,1,3,2,4,3,5,4};
    idx_t weight[]={1,1,1,1,1,1}, parts[6], opts[METIS_NOPTIONS];
    METIS_SetDefaultOptions(opts); opts[METIS_OPTION_SEED]=41; opts[METIS_OPTION_CONTIG]=1;
    auto rc=METIS_PartGraphKway(&nv,&ncon,xadj,adj,weight,nullptr,nullptr,&nparts,
                               nullptr,nullptr,opts,&objective,parts);
    if(rc!=METIS_OK || std::set<idx_t>(parts,parts+6).size()!=3)
      throw std::runtime_error("METIS_PARTITION_FAILED");
    std::cout << "{\"status\":\"PASS\",\"swaths\":" << sorted.size()
              << ",\"swath_length_m\":" << length << ",\"parts\":[";
    for(int i=0;i<6;++i) std::cout << (i ? "," : "") << parts[i];
    std::cout << "],\"edge_cut\":" << objective << "}\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 2; }
}
