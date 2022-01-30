#ifndef UAV_H
#define UAV_H

#include <SFML/Graphics.hpp>
#include <cassert>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>

class UAV : public sf::Drawable, public sf::Transformable
{
public:

    int windowWidth;
    int windowHeight;
    pcl::PointXYZ minPt;
    pcl::PointXYZ maxPt;
    float scaling;

    UAV(int numPoints, int width, int height);
    bool loadUAV (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float pcl::_PointXYZ::* _dim1, float pcl::_PointXYZ::* _dim2);
    float getScalingFactor();
    void resize(int size);
    sf::Vector2f translation();

private:

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    sf::VertexArray m_vertices;
    sf::Texture m_texture;
    float pcl::_PointXYZ::* dim1;
    float pcl::_PointXYZ::* dim2;
};

void sectionGenerationGUI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif