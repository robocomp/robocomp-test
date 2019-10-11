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

/**
       \brief
       @author authorname
*/

// THIS IS AN AGENT


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool AGMCommonBehavior_activateAgent(const ParameterMap &prs);
	bool AGMCommonBehavior_deactivateAgent();
	StateStruct AGMCommonBehavior_getAgentState();
	ParameterMap AGMCommonBehavior_getAgentParameters();
	bool AGMCommonBehavior_setAgentParameters(const ParameterMap &prs);
	void AGMCommonBehavior_killAgent();
	int AGMCommonBehavior_uptimeAgent();
	bool AGMCommonBehavior_reloadConfigAgent();
	void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w);
	void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);
	void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);

public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innerModel;
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerModelViewer;
#endif
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	void regenerateInnerModelViewer();
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);

};

#endif
