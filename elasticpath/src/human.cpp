#include "human.h"
#include <QtCore>
#include <QGraphicsSceneMouseEvent>

Human::Human(const QRectF &r, SocialNavigationGaussianPrxPtr proxy, QGraphicsScene *scene_, QColor color_, QPointF pos) : 
	QGraphicsEllipseItem(r) , gaussian_proxy(proxy), scene(scene_), color(color_)
{
	setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
	QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
    pixmapItem = new QGraphicsPixmapItem( pixmap);
	ellipseItem = new QGraphicsEllipseItem(r);
	ellipseItem->setParentItem(this);
	ellipseItem->setPen(QColor(0,0,0,0));  //transparent
	ellipseItem->setBrush(QColor(0,0,0,0));  //transparent
	pixmapItem->setParentItem(ellipseItem);
	pixmapItem->setPos(this->x()-pixmap.width()/2, this->y()-pixmap.height()/2);
	this->setPos(pos);
	this->color.setAlpha(80);
	this->setZValue(10);
//	updatePolygon( this->pos().x(), this->pos().y(), ellipseItem->rotation());
};

///////////////////////////////////////////////////////////////////////////////////////7
///// Qt Events
////////////////////////////////////////////////////////////////////////////////////////

void Human::mousePressEvent(QGraphicsSceneMouseEvent  *event)
{
	mouseButton = event->button();
	QGraphicsItem::mousePressEvent(event);
}
void Human::mouseReleaseEvent(QGraphicsSceneMouseEvent  *event)
{
	// updatePolygon(pos().x(), pos().y(), ellipseItem->rotation());
	QGraphicsItem::mouseReleaseEvent(event);
}
void Human::mouseMoveEvent(QGraphicsSceneMouseEvent  *event)
{
	if(mouseButton == Qt::RightButton)
	{
  		float angleDegree = atan2(event->pos().x(),event->pos().y()) * -360/(2*M_PI) + 180;
		angleDegree = std::clamp(angleDegree, 0.f, 360.f);
		ellipseItem->setRotation(angleDegree);
	}
//	updatePolygon(pos().x(), pos().y(), ellipseItem->rotation());
	emit personChangedSignal(this);
	QGraphicsItem::mouseMoveEvent(event);
}

void Human::updatePolygon(float x, float y, float ang)
{
	x = x/1000.f; y = y/1000.f;
	ang = ellipseItem->rotation()  * 2*M_PI / 360;
	ang = -ang + M_PI;  // para el social
	RoboCompSocialNavigationGaussian::SNGPerson p{ x, y, ang, 0.f, 0 };
	RoboCompSocialNavigationGaussian::SNGPersonSeq persons{ p };
	try
	{
		auto personal_spaces = gaussian_proxy->getPersonalSpace(persons, 0.8, false);
		if(personal_spaces.size()==0) return;
		if(personal_spaces[0].size()==0) return;
	
		QPolygonF poly;
		for(auto &&p: personal_spaces[0])
			poly << QPointF(p.x*1000.f,p.z*1000.f);
		if( this->getPolygon() != nullptr)
	 		scene->removeItem(this->getPolygon());
		this->setPolygon(scene->addPolygon(poly, color, QBrush(color)));
				
		
	}
	catch(...)
	{
		qDebug() << __FUNCTION__ << "Error reading personal space from SocialGaussian";
	}
}

void Human::updatePolygon(QPolygonF poly)
{
	if( this->getPolygon() != nullptr)
		scene->removeItem(this->getPolygon());
	this->setPolygon(scene->addPolygon(poly, color, QBrush(color)));
}

