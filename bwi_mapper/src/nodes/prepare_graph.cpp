#include <bwi_mapper/topological_mapper.h>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {
  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }
}
#endif

using namespace bwi_mapper;

void drawElementsFile(const std::string& elements_file, cv::Mat& image,
    const Graph& graph, const nav_msgs::MapMetaData& info) {

  std::ifstream fin(elements_file.c_str());

  YAML::Node node;
  bool put_text = true;
  bool put_all_edges = true;
  std::vector<std::pair<size_t, size_t> > edges;
  std::vector<std::pair<size_t, float> > arrows;
  std::vector<int> circles, squares;
#ifdef HAVE_NEW_YAMLCPP
  std::vector<YAML::Node> node_vec = YAML::LoadAll(fin);
  for(std::vector<YAML::Node>::iterator it = node_vec.begin(); it != node_vec.end(); it++) {
    node = *it;
#else
  YAML::Parser parser(fin);
  while(parser.GetNextDocument(node)) {
#endif

    // Get Edges
#ifdef HAVE_NEW_YAMLCPP
    if (node["edges"]) {
#else
    if (node.FindValue("edges")) {
#endif
      put_all_edges = false;
      const YAML::Node& edges_node = node["edges"];
      for(unsigned i = 0; i < edges_node.size(); ++i) {
        std::pair<size_t, size_t> edge;
        edges_node[i][0] >> edge.first;
        edges_node[i][1] >> edge.second;
        edges.push_back(edge);
      }
    }

    // Get Circles
#ifdef HAVE_NEW_YAMLCPP
    if (node["circles"]) {
#else
    if (node.FindValue("circles")) {
#endif
      const YAML::Node& circles_node = node["circles"];
      for(unsigned i = 0; i < circles_node.size(); ++i) {
        int circle;
        circles_node[i] >> circle;
        circles.push_back(circle);
      }
    }

    // Get Squares
#ifdef HAVE_NEW_YAMLCPP
    if (node["squares"]) {
#else
    if (node.FindValue("squares")) {
#endif
      const YAML::Node& squares_node = node["squares"];
      for(unsigned i = 0; i < squares_node.size(); ++i) {
        int square;
        squares_node[i] >> square;
        squares.push_back(square);
      }
    }

    // Get Directional arrows
#ifdef HAVE_NEW_YAMLCPP
    if (node["arrows"]) {
#else
    if (node.FindValue("arrows")) {
#endif
      const YAML::Node& arrows_node = node["arrows"];
      for(unsigned i = 0; i < arrows_node.size(); ++i) {
        std::pair<size_t, float> arrow;
        arrows_node[i][0] >> arrow.first;
        size_t direction;
        arrows_node[i][1] >> direction;
        arrow.second = ((2 * M_PI) / 16) * direction + M_PI/2;
        arrows.push_back(arrow);
      }
    }

#ifdef HAVE_NEW_YAMLCPP
    if (node["put_text"]) {
#else
    if (node.FindValue("put_text")) {
#endif
      const YAML::Node& put_text_node = node["put_text"];
      put_text_node >> put_text;
    }
  }

  bwi_mapper::drawGraph(image, graph, 0, 0, put_text, 
      put_all_edges, edges);
  for(unsigned i = 0; i < circles.size(); ++i) {
    drawCircleOnGraph(image, graph, circles[i]);  
  }
  for(unsigned i = 0; i < squares.size(); ++i) {
    drawSquareOnGraph(image, graph, squares[i]);//, cv::Scalar(0,0,255),0,0,30,-1);  
  }
  for(unsigned i = 0; i < arrows.size(); ++i) {
    drawArrowOnGraph(image, graph, arrows[i], info.width, info.height);  
  }

}

int main(int argc, char** argv) {

  if (argc < 4) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <yaml-graph-file> <yaml-elements-file>" << std::endl;
    return -1;
  }

  bwi_mapper::TopologicalMapper mapper(argv[1]);
  bwi_mapper::Graph graph;
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);
  bwi_mapper::readGraphFromFile(argv[2], info, graph);

  cv::Mat image;
  mapper.drawMap(image);
  drawElementsFile(argv[3], image, graph, info);

  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  cv::imshow("Display window", image);                
  cv::imwrite("out.png", image);                

  cv::waitKey(0); // Wait for a keystroke in the window
  return 0;
}

