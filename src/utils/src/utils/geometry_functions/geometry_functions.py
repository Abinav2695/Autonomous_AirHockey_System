#!/usr/bin/python3
# implement the helper functions similar to 'ssl_common/geometry.hpp'

import math
from ctypes import *
#from geometry_msgs.msg import Pose2D

PI = 3.14159265358979323
INF = 9999999

INVALID_X,INVALID_Y = 999,999
NOT_IN_RANGE_X,NOT_IN_RANGE_Y = 1999,1999

class Vector2D(Structure):
	
	_fields_ =	[ ("x", c_float),
			 ("y", c_float) ]

	def __init__(self,x = None,y = None):
		if x is None:
			self.x = self.y = INF
		elif isinstance(x,Vector2D):
			self.x, self.y = x.x, x.y
		else:
			self.x,self.y=x,y

	def valid(self):
		if math.fabs(self.x) == INF or math.fabs(self.y) == INF :
			return False
		else :
			return True

	# Normalizes the angle (in radians) to be in the range (-pi, pi]
	def normalizeAngle(self,angle):
		if angle > PI :
			return angle - 2 * PI
		elif angle <= -PI :
			return angle + 2 * PI
		else :
			return angle

	# Sets the vector using polar coordinates
	def fromPolar(self,r,theta):
		v = Vector2D(0,0)
		v.x = r*math.cos(theta)
		v.y = r*math.sin(theta)
		return v

	# Returns the absolute value of the vector
	def abs(self,v):
		return math.sqrt(v.x*v.x+v.y*v.y)
	
	def magnitude(self):
		return math.sqrt(self.x*self.x + self.y*self.y)

	# Returns the squared absolute value of the vector
	def absSq(self,v):
		return (v.x*v.x+v.y*v.y)

	# Returns the angle made by the vector (head - self) in the range -pi to pi
	def angle(self,head=None):
		if head is None:
			return math.atan2(self.y,self.x)
		else:
			return math.atan2(head.y-self.y,head.x-self.x)

	def tan_inverse(self):
		if math.atan2(self.y,self.x) < 1.5707963 and math.atan2(self.y,self.x) > -1.5707963:
			return math.atan2(self.y,self.x)
		if math.atan2(self.y,self.x) > 1.5707963 and math.atan2(self.y,self.x) < 3.14159265:
			return math.atan2(self.y,self.x)-3.14159265
		else:
			return math.atan2(self.y,self.x)+3.14159265

	# Returns the Eucledian distance between the 2 vectors
	def dist(self,another_point):
		return math.sqrt(math.pow(another_point.x-self.x,2)+math.pow(another_point.y-self.y,2))

	# Returns the squared Eucledian distane between 2 vectors
	def distSq(self,another_point):
		return math.pow(another_point.x-self.x,2)+math.pow(another_point.y-self.y,2)

	def dot(self,another_point):
		return self.x*another_point.x + self.y*another_point.y

	def __eq__(self,another_point):
		if self.x == another_point.x and self.y == another_point.y :
			return True
		else :
			return False

	def __ne__(self,another_point):
		if self.x == another_point.x and self.y == another_point.y :
			return False
		else :
			return True
	
	def __norm__(self):
		mag =self.magnitude()
		if(mag==0):
			return(Vector2D(0,0))
		else:
			return Vector2D(self.x/mag,self.y/mag)

	def __add__(self,another_point):
		return Vector2D(self.x+another_point.x,self.y+another_point.y)

	def __sub__(self,another_point):
		return Vector2D(self.x-another_point.x,self.y-another_point.y)

	def __mul__(self,scale):
		return Vector2D(self.x * scale , self.y * scale)

	def __truediv__(self,scale):
		if scale == 0 :
			raise Exception('Tried scaling down vector by zero')
		else :
			return Vector2D(self.x/scale,self.y/scale)
	
	# self is the point to be checked if it is within the circle with the center and radius as provided
	# def intersects(self,center,radius):
	# 	if self.distSq(center) < radius * radius :
	# 		return True
	# 	else :
	# 		return False

	# self is the center of circle, checks whether the line made by the point1 and point2 intersects the circle
	# def intersects(self,point1,point2,radius):
	# 	# Source of algorithm used: http://stackoverflow.com/questions/1073336/circle-line-collision-detection 
	# 	d = point2 - point1
	# 	f = point1 - self
	# 	a = d.dot(d)
	# 	b = 2*f.dot(d)
	# 	c = f.dot(f) - radius*radius
	# 	dis = b*b - 4*a*c
	# 	if dis >=0 :
	# 		dis = math.sqrt(dis)
	# 		#print(a,b,c)
	# 		t1 = (dis-b)/(2*a)
	# 		t2 = (-dis-b)/(2*a)
	# 		return t1,t2
	# 		if t1 >= 0 and t1 <= 1 or t2 >= 0 and t2 <=1 :
	# 			return True
	# 	return False
    
    ###############################################
	#intersection of line and cirle equations
	# y=mx+c and sq(x-p)+sq(y-q)=r*r
	#solving eqations we get
	#Ax*x +Bx+C=0
	#A=(m*m+1)
	#B=2(mc−mq−p)
	#C=q*q − r*r + p*p − 2cq + c*c

	#self is the center of the circle 
	def intersects(self,point1,point2,radius):
		# Source of algorithm used: http://stackoverflow.com/questions/1073336/circle-line-collision-detection 
		m,c = point1.slope_const(point2) #slope of line formed by point1 and point2
		p,q=self.x,self.y
		r=radius

		A=(m*m+1)
		B=2*((m*c)-(m*q)-(p))
		C=(q*q - r*r + p*p - 2*c*q + c*c)

		dis = B*B - 4*A*C
		if dis >=0 :
			dis = math.sqrt(dis)
			#print(a,b,c)
			x1 = (dis-B)/(2*A)
			x2 = (-dis-B)/(2*A)
			y1 = m*x1+c
			y2 = m*x2 +c
			if(x1>0):
				return Vector2D(x1,y1)
			elif(x2>0):
				return Vector2D(x2,y2)
			
		return Vector2D(INVALID_X,INVALID_Y)
	

	#self is the center of the circle
	def intersects_slope(self,m,c,radius):
		# Source of algorithm used: http://stackoverflow.com/questions/1073336/circle-line-collision-detection 
		#m,c = point1.slope_const(point2) #slope of line formed by point1 and point2
		p,q=self.x,self.y
		r=radius

		A=(m*m+1)
		B=2*((m*c)-(m*q)-(p))
		C=(q*q - r*r + p*p - 2*c*q + c*c)

		dis = B*B - 4*A*C
		if dis >=0 :
			dis = math.sqrt(dis)
			#print(a,b,c)
			x1 = (dis-B)/(2*A)
			x2 = (-dis-B)/(2*A)
			y1 = m*x1+c
			y2 = m*x2 +c
			if(x1>0 and x2>0):
				if(x1>x2):
					return Vector2D(x1,y1)
				else:
					return Vector2D(x2,y2)
			elif(x2>0):
				return Vector2D(x2,y2)
			elif(x1>0):
				return Vector2D(x1,y1)

		return Vector2D(-1,-1)

    #slope and y intercept of line formedby point1 and point2
	def slope_const(self,another_point):
		slope = (another_point.y-self.y)/(another_point.x-self.x)
		constant =self.y-(slope * self.x)
		return slope,constant


class Line(object):

	def __init__(self, point1=None, angle=None, slope =None,point2=None):

		if not isinstance(point1, Vector2D):
			raise ValueError("point1 should be of type Vector2D, got %s" %type(point1).__name__)
		self.point = point1
		if slope is None:

			if angle is None:
				if not isinstance(point2, Vector2D):
					raise ValueError("point1 should be of type Vector2D, got %s" %type(point2).__name__)
				self.angle = math.atan2(point2.y - point1.y, point2.x - point1.x)
			else:
				self.angle = angle
			if self.angle > math.pi:
				self.angle = math.pi - self.angle
			elif self.angle < -math.pi:
				self.angle = math.pi + self.angle
			self.slope = math.tan(self.angle)
		else:
			self.slope = slope
			self.angle = math.atan(self.slope)


	def intersection_with_line(self, line2):
		if not isinstance(line2,Line):
			raise ValueError("Expected Line instance, got %s" %type(line2).__name__)
		c1 = self.point.y - self.slope * self.point.x
		c2 = line2.point.y - line2.slope * line2.point.x

		m1 = self.slope
		m2 = line2.slope
		P = Vector2D()
		try:
			P.x = int((c2 - c1) / (m1 - m2))
			P.y = int((m1 * c2 - m2 * c1) / (m1 - m2))
			return P
		except:
			return None

	def nearest_point_on_line(self,point):
		
		t=(point.y-self.point.y)*math.sin(self.angle)+(point.x-self.point.x)*(math.cos(self.angle))
		x1=self.point.x+math.cos(self.angle)*t
		y1=self.point.y+math.sin(self.angle)*t
		point=Vector2D(x1,y1)
		return point

	def distance_from_point(self, point):
		# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		# y = mx + c ---> mx - y + c = 0
		if not isinstance(point, Vector2D):
			raise ValueError("Expected Vector2D, got %s" %type(point).__name__)
		p = point
		a = self.slope
		b = -1
		c = self.point.y - self.slope * self.point.x

		distance = math.fabs(a * p.x + b * p.y + c) / math.sqrt(a**2 + b**2)
		return distance


class Circle(object):

	def __init__(self, center=Vector2D(0, 0), radius=0):
		if not isinstance(center, Vector2D):
			raise ValueError("point1 should be of type Vector2D, got %s" %type(center).__name__)
		self.center = center
		self.radius = radius
	
	def if_intersect_with_line(self, line):
		if not isinstance(line, Line):
			raise ValueError("Expected instance of type Line, got %s" %type(line).__name__)
		distance = line.distance_from_point(self.center)
		return distance < self.radius

	def intersection_with_line(self, line):
		if not isinstance(line, Line):
			raise ValueError("Expected instance of type Line, got %s" %type(line).__name__)
		if not self.if_intersect_with_line(line):
			#raise ValueError("Line and Circle doesn't intersect")
			return None
		C = self.center
		R = self.radius
		L = line
		theta = L.angle
		M = L.nearest_point_on_line(C)
		#print(M.x,M.y)
		d = L.distance_from_point(C)
		r = math.sqrt(R**2 - d**2)
		#print(d)
		# A = Vector2D(M.x + r * math.cos(theta), M.y + r * math.sin(theta))
		# B = Vector2D(M.x - r * math.cos(theta), M.y - r * math.sin(theta))
		A, B = Vector2D(), Vector2D()
		A.x = float(M.x + r * math.cos(theta))
		A.y = float(M.y + r * math.sin(theta))
		B.x = float(M.x - r * math.cos(theta))
		B.y = float(M.y - r * math.sin(theta))

		#print(A.x,A.y,B.x,B.y)
		if(A.x<0):
			return None
		return A
			# only one value which is positive and incase both are positive then the position with higher y value

	def if_point_in_circle(self, point):
		if not isinstance(point, Vector2D):
			raise ValueError("Expected Vector2D() got %s" %type(point).__name__)
		distance = self.center.dist(point)
		return distance < self.radius

if __name__ == "__main__":
	pointX = Vector2D(5,5)
	print(pointX.__norm__().x)
	Line1 = Line(point1 =Vector2D(592.4545288085938 ,191.40908813476562),point2=Vector2D(627.3673706054688, 154.16326904296875))  
	# Line2 = Line(point1 = Vector2D(0,11),point2=Vector2D(11,11))
	# print(Line1.slope)
	# print((Line1.intersection_with_line(Line2)))
	myCircle = Circle(radius=55,center =Vector2D(0,151))
	p1 = Line1.nearest_point_on_line(Vector2D(0,151))
	print(p1.x,p1.y)