#!/usr/bin/python3
from utils.geometry_functions.geometry_functions import Vector2D as point2D
from utils.geometry_functions.geometry_functions import Line as line2D
from utils.geometry_functions.geometry_functions import Circle as circle2D
import utils.config.config as constants
import cv2

class Path_Prediction:
    #def __init__(self,TABLE_WIDTH_PIXEL,TABLE_HEIGHT_PIXEL,TABLE_WIDTH_PIXEL_MIN,TABLE_HEIGHT_PIXEL_MIN):
    def __init__(self,TABLE_WIDTH_MM=constants.table["TABLE_WIDTH_IN_MM"],TABLE_HEIGHT_MM=constants.table["TABLE_HEIGHT_IN_MM"],
                                            TABLE_WIDTH_MM_MIN=0,TABLE_HEIGHT_MM_MIN=0):
        self.TABLE_WIDTH_MM = TABLE_WIDTH_MM
        self.TABLE_HEIGHT_MM = TABLE_HEIGHT_MM

        self.TABLE_WIDTH_MM_MIN = TABLE_WIDTH_MM_MIN
        self.TABLE_HEIGHT_MM_MIN = TABLE_HEIGHT_MM_MIN

        self.xAxis = line2D(point1 = point2D(self.TABLE_WIDTH_MM_MIN,self.TABLE_HEIGHT_MM_MIN),
                                            point2=point2D(self.TABLE_WIDTH_MM_MIN,self.TABLE_HEIGHT_MM))
        self.yAxis = line2D(point1=point2D(self.TABLE_WIDTH_MM_MIN,self.TABLE_HEIGHT_MM_MIN),point2 = point2D(self.TABLE_WIDTH_MM,self.TABLE_HEIGHT_MM_MIN))
        self.otherEdgeOfTable = line2D(point1=point2D(self.TABLE_WIDTH_MM_MIN,self.TABLE_HEIGHT_MM),point2 = point2D(self.TABLE_WIDTH_MM,self.TABLE_HEIGHT_MM))

    

    def intersection_point_finder_for_line(self,puckLineOfMotion,lineOfIntersection,predictionCount,newPuckPoint,totalDistance):
        """[summary]

        Args:
            puckLineOfMotion ([type]): [description]
            lineOfIntersection ([type]): [description]
            predictionCount ([type]): [description]
            newPuckPoint ([type]): [description]
            totalDistance ([type]): [description]

        Returns:
            [type]: [description]
        """
        intersectionPoint = puckLineOfMotion.intersection_with_line(lineOfIntersection)
        predictionCount+=1
        #print(intersectionPoint.x,intersectionPoint.y)
        if intersectionPoint is None:
            
            return (constants.INVALID_POINT,0)
            
        else:
                ### Falls within table .. puck is moving head on 
            if(self.TABLE_HEIGHT_MM_MIN < intersectionPoint.y < self.TABLE_HEIGHT_MM):
                # intersectionPointWithTableEdge = puckLineOfMotion.intersection_with_line(self.xAxis)
                # intoGoal=0
                # if((0.3*self.TABLE_HEIGHT_MM)<intersectionPointWithTableEdge.y<(0.7*self.TABLE_HEIGHT_MM)):
                #     intoGoal=1
                # else:
                #     intoGoal=0
                totalDistance+=intersectionPoint.dist(newPuckPoint)
                return (intersectionPoint,totalDistance)
            elif(predictionCount>2):
                
                return (constants.INVALID_POINT,totalDistance)
            else:
                newSlope = -puckLineOfMotion.slope
                
                pointForReboundLine=None
                if(puckLineOfMotion.slope>0):
                    pointForReboundLine = puckLineOfMotion.intersection_with_line(self.yAxis)
                else:
                    pointForReboundLine = puckLineOfMotion.intersection_with_line(self.otherEdgeOfTable)
                
                if pointForReboundLine is None:                  
                    return (constants.INVALID_POINT,totalDistance)
                newLineOfMotion = line2D(point1=pointForReboundLine,slope=newSlope)
                totalDistance+=pointForReboundLine.dist(newPuckPoint)
                return self.intersection_point_finder_for_line(newLineOfMotion,lineOfIntersection,predictionCount,pointForReboundLine,totalDistance)


    def intersection_point_finder_for_circle(self,puckLineOfMotion,circleOfIntersection,predictionCount,newPuckPoint,totalDistance):
        """[summary]

        Args:
            puckLineOfMotion ([type]): [description]
            lineOfIntersection ([type]): [description]
            predictionCount ([type]): [description]
            newPuckPoint ([type]): [description]
            totalDistance ([type]): [description]

        Returns:
            [type]: [description]
        """
        intersectionPoint = circleOfIntersection.intersection_with_line(puckLineOfMotion)
        predictionCount+=1
        #print(puckLineOfMotion,circleOfIntersection)
        if intersectionPoint is None:
            if(predictionCount>2):
                return (constants.INVALID_POINT,totalDistance)
            
            else:
                newSlope = -puckLineOfMotion.slope
                pointForReboundLine=None
                if(puckLineOfMotion.slope>0):
                    pointForReboundLine = puckLineOfMotion.intersection_with_line(self.yAxis)
                else:
                    pointForReboundLine = puckLineOfMotion.intersection_with_line(self.otherEdgeOfTable)
                
                if pointForReboundLine is None:                  
                    return (constants.INVALID_POINT,totalDistance)
                newLineOfMotion = line2D(point1=pointForReboundLine,slope=newSlope)
                totalDistance+=pointForReboundLine.dist(newPuckPoint)
                return self.intersection_point_finder_for_circle(newLineOfMotion,circleOfIntersection,predictionCount,pointForReboundLine,totalDistance)
        
        else:
            # intersectionPointWithTableEdge = puckLineOfMotion.intersection_with_line(self.xAxis)
            # intoGoal=0
            # if((0.3*self.TABLE_HEIGHT_MM)<intersectionPointWithTableEdge.y<(0.7*self.TABLE_HEIGHT_MM)):
            #     intoGoal=1
            # else:
            #     intoGoal=0
            totalDistance+=intersectionPoint.dist(newPuckPoint)
            return (intersectionPoint,totalDistance)

    def find_puck_predicted_point(self,newPoint,previousPoint,intersection_line_x=constants.table["DEFEND_LINE_IN_PIXEL"],
                            intersectionCircleRadius=constants.table["DEFEND_CIRCLE1_RADIUS_IN_PIXEL"],
                            intersectionCircleCenter= constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"],isLine=1):
        
        
        intersection_line_x_in_mm = intersection_line_x / constants.table["PIXEL_TO_MM_RATIO"]
        intersectionCircleRadius_in_mm = intersectionCircleRadius /constants.table["PIXEL_TO_MM_RATIO"]
        intersectionCircleCenter_in_mm =point2D()
        intersectionCircleCenter_in_mm.x = intersectionCircleCenter.x /constants.table["PIXEL_TO_MM_RATIO"]
        intersectionCircleCenter_in_mm.y = intersectionCircleCenter.y / constants.table["PIXEL_TO_MM_RATIO"]

        ##find required lines
        puckLineOfMotion = line2D(point1 =newPoint,point2=previousPoint)
        if(isLine):            
            lineOfIntersection = line2D(point1 =point2D(intersection_line_x_in_mm,self.TABLE_HEIGHT_MM_MIN),point2=point2D(intersection_line_x_in_mm,self.TABLE_HEIGHT_MM))         
            return self.intersection_point_finder_for_line(puckLineOfMotion,lineOfIntersection,0,newPoint,0)
        
        else:
            circleOfIntersection = circle2D(center=intersectionCircleCenter_in_mm,radius=intersectionCircleRadius_in_mm)
            return self.intersection_point_finder_for_circle(puckLineOfMotion,circleOfIntersection,0,newPoint,0)

        
if __name__=="__main__":
    pass