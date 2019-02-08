#include "human.h"
#include <QtCore>
#include <QGraphicsSceneMouseEvent>

Human::Human(const QRectF &r, SocialNavigationGaussianPrxPtr proxy) : QGraphicsEllipseItem(r) 
{
    qDebug() << "creado";
    gaussian_proxy = proxy;

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
	QGraphicsItem::mousePressEvent(event);
}
void Human::mouseReleaseEvent(QGraphicsSceneMouseEvent  *event)
{
	SNGPerson p;
	p.x = this->pos().x();
	p.z = this->pos().y();
	p.angle = this->rotation();
	SNGPersonSeq persons;
	persons.push_back(p);
	try{
		//intimate_seq
		auto points = gaussian_proxy->getPersonalSpace(persons, 0.8, false);
		SNGPolylineSeq intimate_seq;
		for (auto p:points){
			intimate_seq.push_back(p);
		}
		emit personChangedSingal(intimate_seq);
	}catch(...)
	{
		qDebug()<<"Error reading personal space from SocialGaussian";
	}
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
	QGraphicsItem::mouseMoveEvent(event);
}

/*QVariant Human::itemChange(GraphicsItemChange change, const QVariant &value)
{
	switch (change) 
	{
    	case ItemPositionHasChanged:
			// If human moves update polyline by calling the proxy
			
			// Maybe in the mouserelease event
			qDebug() << "gola";
        	break;
	    default:
    	    break;
    };
}*/
