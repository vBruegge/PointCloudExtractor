#ifndef UAV_H
#define UAV_H

#include <SFML/Graphics.hpp>
#include <cassert>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <string>

class UAV : public sf::Drawable, public sf::Transformable
{
public:

    int windowWidth;
    int windowHeight;
    pcl::PointXYZ minPt;
    pcl::PointXYZ maxPt;
    float scaling;

    /**
     * @brief Construct a new UAV object
     * 
     * @param numPoints number of points in the point cloud
     * @param width window width
     * @param height window height
     */
    UAV(int numPoints, int width, int height);

    /**
     * @brief loads the 3D-points of the point cloud into the drawable vertices of the UAV class
     * 
     * @param cloud point cloud which should be printed
     * @param _dim1 first dimension of the point cloud (e.g. X)
     * @param _dim2 second dimension of the point cloud (e.g. Y)
     */
    void loadUAV (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float pcl::_PointXYZ::* _dim1, float pcl::_PointXYZ::* _dim2);
    
    /**
     * @brief Get the scaling factor
     * 
     * @return float scaling factor
     */
    float getScalingFactor();

    /**
     * @brief resize the drawable vertices vector
     * 
     * @param size new number of points
     */
    void resize(int size);

    /**
     * @brief translate the printed UAV
     * 
     * @return sf::Vector2f translation
     */
    sf::Vector2f translation();

private:

    /**
     * @brief draws the GUI
     * 
     * @param target 
     * @param states 
     */
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    sf::VertexArray m_vertices;
    sf::Texture m_texture;
    float pcl::_PointXYZ::* dim1;
    float pcl::_PointXYZ::* dim2;
};

/**
 * @brief draws different sights of the uav to determine the distances where to cut
 * 
 * @param cloud which is sectioned
 * @param sourceFolder path to writeable folder including the point cloud
 * @param filename name of the point cloud
 */
void sectionGenerationGUI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string sourceFolder, std::string filename);

#endif