/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

#ifdef USE_QTGUI
	innerModelViewer = NULL;
	osgView = new OsgView(this);
    this->setMinimumSize(800,800);
    osgView->setMinimumSize(800,800);
    osgView->getCamera()->setViewport(new osg::Viewport(0, 0, 800, 800));
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(0., 9000., -17000.));
	osg::Vec3d center(osg::Vec3(0., 0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, false);
	osgView->setCameraManipulator(tb);
#endif
    
  ListDigraph g;
  ListDigraph::Node u = g.addNode();
  ListDigraph::Node v = g.addNode();
  ListDigraph::Arc  a = g.addArc(u, v);
  cout << "Hello World! This is LEMON library here." << endl;
  cout << "We have a directed graph with " << countNodes(g) << " nodes "
       << "and " << countArcs(g) << " arc." << endl;
    
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }


#ifdef USE_QTGUI
	innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
#endif
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
}

void SpecificWorker::compute()
{

#ifdef USE_QTGUI
    guard gl(mutex);
    innerModel->update();
    if (innerModelViewer)
    { 
        
        innerModelViewer->update();
    }
	osgView->frame();
#endif
}


//Receive humanlist publicacion
void SpecificWorker::HumanPose_obtainHumanPose(const humansDetected &list_of_humans)
{
    guard gl(mutex);
    std::cout << "humanlist received: "<<list_of_humans.size() << std::endl;
    for(auto &human: list_of_humans)
    {
        if(humans.contains(QString::number(human.id)))
            qDebug()<<"human is already added to innerModel, change its position instead";
        else
            add_human_innermodel(human);
    }
    qDebug()<<"humans"<<humans.keys();
    //just for testing pourpose ==> empty list remove all humans
    if(list_of_humans.size() == 0)
    {
        for (auto &person: humans.values())
        {
            qDebug()<<"trying to remove"<<QString::number(person.id);
            remove_human_innermodel(person);
        }
    }
    qDebug()<<"************************";
}


//UTILS
void SpecificWorker::add_human_innermodel(PersonType person)
{
    try
    {
        QString name = QString::number(person.id);
     	QString meshPath = QString("/home/robocomp/robocomp/components/robocomp-shelly/files/mesh/human01.3ds");
        InnerModelNode* room = innerModel->getNode("room");
        InnerModelTransform* transform = innerModel->newTransform(name, "static", room, person.pos.x, 0, person.pos.z, 0, person.pos.ry, 0, 0);
        room->addChild(transform);    
        InnerModelMesh* mesh = innerModel->newMesh(name+"_mesh", transform, meshPath, 12, 0, 0, 0, 1.5708, 0, 3.1416, false);
        transform->addChild(mesh);
        innerModelViewer->recursiveConstructor(transform);
        humans[name] = person;
        qDebug()<<"Added human:"<<name;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void SpecificWorker::remove_human_innermodel(PersonType person)
{
    try
    {
        QString name = QString::number(person.id);
        InnerModelNode* node = innerModel->getNode(name);
        QStringList l;
        innerModelViewer->recursiveRemove(node);
        innerModel->removeSubTree(node, &l);
        humans.remove(name);
        qDebug()<<"Remove human: "<<name;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}


float SpecificWorker::euclideanD(float x1, float z1, float x2, float z2)
{
    return sqrt((x1-x2)*(x1-x2) + (z1-z2)*(z1-z2));
}

