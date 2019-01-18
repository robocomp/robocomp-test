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
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	innerModelViewer = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	osgView->setCameraManipulator(tb);

	innerModel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
	innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
	excludedNodes.insert("ddG");
	recursiveIncludeMeshes(innerModel->getRoot(), "base", false, robotNodes, restNodes, excludedNodes);
	for(auto s : robotNodes)
		qDebug() << s;
	qDebug() << "--------------------------";
	for(auto s : restNodes)
		qDebug() << s;
		
	timer.start(100);
}

void SpecificWorker::compute()
{
	// if (innerModelViewer) innerModelViewer->update();
	// 	osgView->frame();
	
	for(auto &in : robotNodes )
		for ( auto &out : restNodes )
		{
			if ( innerModel->collide( in, out))
				qDebug() << in << " collision with " << out;
		}
}

/**
 * @brief Constructs the list of robot and world meshes from InnerModel that will be used in detection of collisions
 * 
 * @param node pointer to node used to traverse the tree
 * @param robotId robot's tag name
 * @param inside flag
 * @param in growing list of mesh names belonging to the robot
 * @param out growing list of mesh names belonging to the world
 * @param excluded list of meshes to be excluded from both lists
 * @return void
 */
void SpecificWorker::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out, std::set<QString> &excluded)
{	
	if (node->id == robotId)
		inside = true;
	
	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;
	
	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))  
		for (int i=0; i<node->children.size(); i++)
			recursiveIncludeMeshes(node->children[i], robotId, inside, in, out, excluded);	
	
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		if( std::find(excluded.begin(), excluded.end(), node->id) == excluded.end() )			
		{
			if (inside)
				in.push_back(node->id);
			else
				if(mesh or plane)
					out.push_back(node->id);
		}
	}
}
