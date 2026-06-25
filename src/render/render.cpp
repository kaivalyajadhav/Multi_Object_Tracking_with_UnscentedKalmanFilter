#include "render.h"

void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

  double roadLengthAhead = 50.0;
  double roadLengthBehind = -15.0;
  double roadWidth = 12.0;
  double roadHeight = 0.2;

  viewer->addCube(roadLengthBehind, roadLengthAhead, -roadWidth / 2, roadWidth / 2, -roadHeight, 0, .2, .2, .2, "highwayPavement");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "highwayPavement");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highwayPavement");
  viewer->addLine(pcl::PointXYZ(roadLengthBehind, -roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead , -roadWidth / 6, 0.01), 1, 1, 0, "line1");
  viewer->addLine(pcl::PointXYZ(roadLengthBehind, roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead, roadWidth / 6, 0.01), 1, 1, 0, "line2");

  double poleSpace = 10;
  double poleCurve = 4;
  double poleWidth = 0.5;
  double poleHeight = 3;

  double markerPos = (roadLengthBehind/poleSpace)*poleSpace-distancePos;
  while(markerPos < roadLengthBehind)
    markerPos+=poleSpace;
  int poleIndex = 0;
  while(markerPos <= roadLengthAhead)
  {
    viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2+roadWidth/2+poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"l");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"l");
    viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2+roadWidth/2+poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"lframe");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"lframe");

    viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2-roadWidth/2-poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"r");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"r");
    viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2-roadWidth/2-poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"rframe");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"rframe");

    markerPos+=poleSpace;
    poleIndex++;
  }
}

void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  // stub: not yet implemented
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // stub: not yet implemented
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
{
  // stub: not yet implemented
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color)
{
  // stub: not yet implemented
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color, float opacity)
{
  // stub: not yet implemented
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity)
{
  // stub: not yet implemented
}
