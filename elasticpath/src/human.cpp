#include "human.h"
#include <QtCore>
#include <QGraphicsSceneMouseEvent>

Human::Human() : QGraphicsEllipseItem() 
{
    qDebug() << "creado";
    setFlag(QGraphicsItem::ItemIsMovable);
};

///////////////////////////////////////////////////////////////////////////////////////7
///// Qt Events
////////////////////////////////////////////////////////////////////////////////////////

void Human::mousePressEvent(QGraphicsSceneMouseEvent  *event)
{
	if(event->button() == Qt::RightButton)
	{
        qDebug() << "hola";
		// check the item is a person
		// assign mouse y coordinate to current person angle
		// move person angle with mouse y coordinate
		
	}
}