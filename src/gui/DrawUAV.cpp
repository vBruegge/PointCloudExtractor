#include "DrawUAV.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/common/geometry.h>
#include <stdio.h>
#include <fstream>

//initialize drawing object
UAV::UAV(int numPoints, int width, int height) {
    m_vertices.setPrimitiveType(sf::Points);
    m_vertices.resize(numPoints);
    windowWidth = width;
    windowHeight = height;
    scaling = 1;
}

//change attributes of the UAV drawing object
bool UAV::loadUAV (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float pcl::_PointXYZ::* _dim1, float pcl::_PointXYZ::* _dim2) {
    for(int i = 0; i < cloud->size();i++) {
        sf::Vector2f tmp = sf::Vector2f((cloud->points[i].*_dim1), (cloud->points[i].*_dim2));
        m_vertices[i].position = tmp;
        m_vertices[i].color = sf::Color::Yellow;
    }
    dim1 = _dim1;
    dim2 = _dim2;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    return true;
}
float UAV::getScalingFactor(){
    if(abs(minPt.*dim1-maxPt.*dim1) > abs(minPt.*dim2-maxPt.*dim2))
        return windowWidth/(abs(minPt.*dim1-maxPt.*dim1)+200);
    else
        return windowHeight/(abs(minPt.*dim2-maxPt.*dim2)+100);
}
void UAV::resize(int size) {m_vertices.resize(size);}

sf::Vector2f UAV::translation() {
    if(abs(minPt.*dim1) < 1)
        return sf::Vector2f(100.f, (windowHeight-abs(maxPt.*dim2+minPt.*dim2))/(float)2);
    else
        return sf::Vector2f((windowWidth-abs(maxPt.*dim1+minPt.*dim1))/(float)2, (windowHeight-abs(maxPt.*dim2+minPt.*dim2))/(float)2);
    }

void UAV::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    // apply the entity's transform -- combine it with the one that was passed by the caller
    states.transform *= getTransform(); // getTransform() is defined by sf::Transformable

    sf::Transform t = sf::Transform::Identity;
    if(abs(minPt.*dim1) < 1)
        t.translate(100.f, (windowHeight/(float)2));
    else if(maxPt.*dim1 + minPt.*dim1 < -windowWidth)
        t.translate(windowWidth - 100.f, (windowHeight/(float)2));
    else
        t.translate(windowWidth/(float)2, windowHeight/(float)2);
    
    if(abs(minPt.*dim1-maxPt.*dim1) > abs(minPt.*dim2-maxPt.*dim2))
        t.scale(windowWidth/(abs(minPt.*dim1-maxPt.*dim1)+200)*scaling, windowWidth/(abs(minPt.*dim1-maxPt.*dim1)+200)*scaling);
    else
        t.scale(windowHeight/(abs(minPt.*dim2-maxPt.*dim2)+100)*scaling, windowHeight/(abs(minPt.*dim2-maxPt.*dim2)+100)*scaling);
    states.transform = t;

    // apply the texture
    states.texture = NULL;

    // draw the vertex array
    target.draw(m_vertices, states);
}

void sectionGenerationGUI (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string sourceFolder, std::string filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr shortendCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*cloud, min, max);

    std::ofstream fout(filename);

    std::cout << "Does the UAV have a v-tail? (y/n)\n";
    char answer;
    std::cin >> answer;
    int numSections;
    if(answer == 'y') {
        numSections = 4;
        fout << 'v' << std::endl;
    }
    else {
        numSections = 5;
        fout << 'h' << std::endl;
    }

    //init draw of uav
    float split;
    UAV uav((int) cloud->size(), 1920, 1080);

    sf::Font font;
    font.loadFromFile(sourceFolder + "/src/gui/Arial.TTF");

    //explanation of the seperate windows
    sf::Text text;
    text.setFont(font);
    text.setCharacterSize(24);
    text.setFillColor(sf::Color::White);

    //explanation of keys in the GUI
    sf::Text usingHelp;
    usingHelp.setFont(font);
    usingHelp.setCharacterSize(24);
    usingHelp.setFillColor(sf::Color::White);
    usingHelp.move(1450.f,850.f);
    usingHelp.setString("Left click: select a section\nRight click (on section): deselect section\nScrolling: Zoom in/out\nClose Window: Save selection");

    for(int i = 0; i < numSections; i++) {
        sf::RenderWindow window(sf::VideoMode(1920, 1080), "Section-Generation");
        std::vector<sf::Vertex> lineArray;
        switch(i) {
            case 0: //split UAV between wing and tail
                uav.loadUAV(cloud, &pcl::PointXYZ::y, &pcl::PointXYZ::z);
                text.setString("Please half the UAV between the wing and the tail.");
            break;
            case 1: //fuselage sections
                uav.loadUAV(cloud, &pcl::PointXYZ::y, &pcl::PointXYZ::x);
                text.setString("Please choose the wished fuselage sections");
                fout << "fuselage ";
            break;
            case 2: //wing sections
                pass.setInputCloud (cloud);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (0, FLT_MAX);
                pass.filter (*shortendCloud);

                pass.setInputCloud (shortendCloud);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (-FLT_MAX, split);
                pass.filter (*shortendCloud);

                uav.resize(shortendCloud->size());
                uav.loadUAV(shortendCloud, &pcl::PointXYZ::x, &pcl::PointXYZ::z);
                if(abs(uav.maxPt.x) == abs(max.x)) {
                    text.setString("Please choose the wished wing sections");
                    fout << "wing ";
                }
                else {
                    text.setString("Please choose the wished horizontail tail/v-tail sections");
                    fout << "horizontal_tail ";
                }

            break;
            case 3: //horizontal tail and v-tail
                pass.setInputCloud (cloud);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (0, FLT_MAX);
                pass.filter (*shortendCloud);

                pass.setInputCloud (shortendCloud);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (split, FLT_MAX);
                pass.filter (*shortendCloud);

                uav.resize(shortendCloud->size());
                uav.loadUAV(shortendCloud, &pcl::PointXYZ::x, &pcl::PointXYZ::z);
                if(abs(uav.maxPt.x) == abs(max.x)) {
                    text.setString("Please choose the wished wing sections");
                    fout << "wing ";
                }
                else  {
                    text.setString("Please choose the wished horizontail tail/v-tail sections");
                    fout << "horizontal_tail ";
                }
            break;
            case 4: //vertical tail
                pass.setInputCloud (cloud);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (split, FLT_MAX);
                pass.filter (*shortendCloud);

                uav.resize(shortendCloud->size());
                uav.loadUAV(shortendCloud, &pcl::PointXYZ::z, &pcl::PointXYZ::y);
                if(abs(uav.maxPt.x-uav.minPt.x) == abs(max.x-min.x)) {
                    pass.setInputCloud (cloud);
                    pass.setFilterFieldName ("y");
                    pass.setFilterLimits (-FLT_MAX, split);
                    pass.filter (*shortendCloud);

                    uav.resize(shortendCloud->size());
                    uav.loadUAV(shortendCloud, &pcl::PointXYZ::y, &pcl::PointXYZ::z);
                }
                text.setString("Please choose the wished vertical tail sections");
                fout << "vertical_tail ";
            break;
        }

        while (window.isOpen()) {

            sf::Event e;
            while (window.pollEvent(e)) {

                switch (e.type) {
                    case sf::Event::EventType::Closed:
                        //save chosen sections and close window
                        window.close();
                        for(int j = 0; j < lineArray.size(); j+=2) {
                            float tmp = lineArray[j].position.x-uav.translation().x;
                            if(i < 4) {
                                fout << tmp/uav.getScalingFactor() << " ";
                            }
                            else {
                                tmp = lineArray[j].position.y-uav.translation().y;
                                fout << tmp/uav.getScalingFactor() << " ";
                            }
                            if(i == 0)
                                split = tmp/uav.getScalingFactor();
                        }
                        fout << std::endl;
                        break;
                    case sf::Event::EventType::MouseButtonPressed: //select and deselect sections
                        if(i<4) { //vertical sections
                            float location = e.mouseButton.x;
                            location = (-uav.translation().x * (1-uav.scaling) / uav.scaling + location / uav.scaling);
                            if(e.mouseButton.button == sf::Mouse::Left) { //select
                                    lineArray.push_back(sf::Vertex(sf::Vector2f(location, 0.f), sf::Color::Red));
                                    lineArray.push_back(sf::Vertex(sf::Vector2f(location, uav.windowHeight), sf::Color::Red));
                            }
                            else if(e.mouseButton.button == sf::Mouse::Right) { //deselect
                                for(int j = 0; j < lineArray.size(); j+=2) {
                                    if(location < lineArray[j].position.x+5 && location >lineArray[j].position.x-5) {
                                        lineArray.erase(lineArray.begin()+j, lineArray.begin()+j+2);
                                        j-=2;
                                    }
                                }
                            }
                        }
                        else { //horizontal sections
                            float location = e.mouseButton.y;
                            location = (-uav.translation().y * (1-uav.scaling) / uav.scaling + location / uav.scaling);
                            if(e.mouseButton.button == sf::Mouse::Left) { //select
                                    lineArray.push_back(sf::Vertex(sf::Vector2f(0.f, location), sf::Color::Red));
                                    lineArray.push_back(sf::Vertex(sf::Vector2f(uav.windowWidth, location), sf::Color::Red));
                            }
                            else if(e.mouseButton.button == sf::Mouse::Right) { //deselect
                                for(int j = 0; j < lineArray.size(); j+=2) {
                                    if(location < lineArray[j].position.y+5 && location >lineArray[j].position.y-5) {
                                        lineArray.erase(lineArray.begin()+j, lineArray.begin()+j+2);
                                        j-=2;
                                    }
                                }
                            }
                        }
                        break;
                    case sf::Event::EventType::MouseWheelScrolled: //zoom in/zoom out
                        if(e.mouseWheelScroll.delta > 0)
          uav.scaling *= 1.1f;
                        else
                            uav.scaling /= 1.1f;
                        break;
                }
            }

            //draw objects
            window.clear();
            window.draw(uav);
            window.draw(text);
            window.draw(usingHelp);
            for(int j = 0; j < lineArray.size(); j+=2) {
                sf::Vertex line[2] = {lineArray[j], lineArray[j+1]};
                if(i < 4) {
                    line[0].position.x = uav.translation().x * (1-uav.scaling) + line[0].position.x * uav.scaling;
                    line[1].position.x = uav.translation().x * (1-uav.scaling) + line[1].position.x * uav.scaling;
                    window.draw(line, 2, sf::Lines);
                }
                else {
                    line[0].position.y = uav.translation().y * (1-uav.scaling) + line[0].position.y * uav.scaling;
                    line[1].position.y = uav.translation().y * (1-uav.scaling) + line[1].position.y * uav.scaling;
                    window.draw(line, 2, sf::Lines);
                }
            }
            window.display();
        }
    }
}  
