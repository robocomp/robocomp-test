#include "human.h"
#include <QtCore>
#include <QGraphicsSceneMouseEvent>

Human::Human(const QRectF &r) : QGraphicsEllipseItem(r) 
{
    qDebug() << "creado";
    setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
	QPixmap pixmap = QPixmap::fromImage(QImage("/home/robocomp/robocomp/components/robocomp-tests/elasticpath/src/person.png")).scaled(800,400);
    pixmapItem = new QGraphicsPixmapItem( pixmap);
	ellipseItem = new QGraphicsEllipseItem(r);
	ellipseItem->setParentItem(this);
	pixmapItem->setParentItem(ellipseItem);
	pixmapItem->setPos(this->x()-pixmap.width()/2, this->y()-pixmap.height()/2);
};

///////////////////////////////////////////////////////////////////////////////////////7
///// Qt Events
////////////////////////////////////////////////////////////////////////////////////////

void Human::mousePressEvent(QGraphicsSceneMouseEvent  *event)
{
	mouseButton = event->button();
	if(event->button() == Qt::RightButton)
	{	
		//Change color or something
	}
	QGraphicsItem::mousePressEvent(event);
}
void Human::mouseMoveEvent(QGraphicsSceneMouseEvent  *event)
{
	qDebug()<<"move";
	if(mouseButton == Qt::RightButton)
	{
		// assign mouse y coordinate to current person angle
  		float angleDegree = atan2(event->pos().x(),event->pos().y()) * -360/(2*M_PI) + 180;
//		qDebug()<<"y pose"<<event->pos().y();
		//float angleDegree = this->rotation() + event->pos().y()/8;
		angleDegree = std::clamp(angleDegree, 0.f, 360.f);
		qDebug()<<"angle"<<angleDegree;
		ellipseItem->setRotation(angleDegree);
	}
	QGraphicsItem::mouseMoveEvent(event);
}
