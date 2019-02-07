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
	pixmapItem->setParentItem(this);
	pixmapItem->setPos(this->x()-pixmap.width()/2, this->y()-pixmap.height()/2);
	QTransform t;
	t.translate(this->boundingRect().center().x(), this->boundingRect().center().y());
	pixmapItem->setTransform(t);

};

///////////////////////////////////////////////////////////////////////////////////////7
///// Qt Events
////////////////////////////////////////////////////////////////////////////////////////

void Human::mousePressEvent(QGraphicsSceneMouseEvent  *event)
{
	mouseButton = event->button();
	if(event->button() == Qt::RightButton)
	{
		// assign mouse y coordinate to current person angle
		float angle = this->rotation();
		// move person angle with mouse y coordinate
		
	}
	QGraphicsItem::mousePressEvent(event);
}
void Human::mouseMoveEvent(QGraphicsSceneMouseEvent  *event)
{
	qDebug()<<"move";
	if(mouseButton == Qt::RightButton)
	{
		// assign mouse y coordinate to current person angle
// 		float angle = atan2(event->pos().x(),event->pos().y());
// 		// move person angle with mouse y coordinate
// 		float angleDegree = -360/(2*M_PI)*angle + 180;
// 		qDebug()<<angle<<angleDegree;
// 		this->setRotation(angleDegree);
		qDebug()<<"y pose"<<event->pos().y();
		float angleDegree = this->rotation() + event->pos().y()/8;
		if (angleDegree > 360) angleDegree = 360;
		if (angleDegree < 0) angleDegree = 0;
		qDebug()<<"angle"<<angleDegree;
		
// 		float angle = atan2(pos.y(),pos.x())*180/M_PI + 90;
// 		int angle2 = (int(angle)+360) % 360;
// 		qDebug()<<"angle"<<angle;
// 		t.rotate(angle2);
// 		t.translate(-center.x(), -center.y());
// 		
		pixmapItem->setRotation(angleDegree);
		
		
	}
	QGraphicsItem::mouseMoveEvent(event);
}
