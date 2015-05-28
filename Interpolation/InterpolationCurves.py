import maya.cmds as mc

class matrix4x4:
    def __init__(self, inputList=None):
        self.data = [[0 for n in range(4)] for n in range(4)]
        if inputList is not None:
            for m in range(4):
                for n in range(4):
                    self.data[m][n] = inputList[m * 4 + n];
        
    def __repr__(self):
        returnString = "";
        for m in range(4):
            returnString += "[ "
            for n in range(3):
                returnString += str(self.data[m][n]) + ", "
            returnString += str(self.data[m][3]) + " ]\n"
        return returnString;
        
    def __mul__(self, right):
        if isinstance(right, matrix4x4):
            returnValue = [0 for n in range(16)]
            for m in range(4):
                for n in range(4):
                    returnValue[4 * m + n] = 0;
                    for k in range(4):
                        returnValue[4 * m + n] += self.data[m][k] * right.data[k][n];
            return matrix4x4(returnValue);
            
        elif isinstance(right, matrix4x1):
            returnValue = [0 for n in range(4)]
            for m in range(4):
                returnValue[m] = 0;
                for k in range(4):
                    returnValue[m] += self.data[m][k] * right.data[k];
            return matrix4x1(returnValue);
            
        elif isinstance(right, float) or isinstance(right, int):
            returnValue = [0 for n in range(16)]
            for m in range(4):
                for n in range(4):
                    returnValue[4 * m + n] = self.data[m][n] * right;
            return matrix4x4(returnValue);
            
        else:
            print "not correct type"
            return;
            
    def __rmul__(self, left):
        return self.__mul__(left);
            
            
            
class matrix4x1:
    def __init__(self, inputList=None):
        self.data = [0 for n in range(4)]
        if inputList is not None:
            for m in range(4):
                self.data[m] = inputList[m];
        
    def __repr__(self):
        returnString = ""
        for m in range(4):
            returnString += "[ " + str(self.data[m]) + " ]\n"
        return returnString;
        
    def __mul__(self, right):
        if isinstance(right, matrix1x4):
            returnValue = [0 for x in range(16)]
            for m in range(4):
                for n in range(4):
                    returnValue[4 * m + n] = self.data[m] * right.data[n];            
            return matrix4x4(returnValue);
            
        elif isinstance(right, float) or isinstance(right, int):
            returnValue = [0 for x in range(4)];
            for m in range(4):
                returnValue[m] = right * self.data[m];
            return matrix4x1(returnValue);
            
        else:
            print "not correct type"
            return;
            
    def __rmul__(self, left):
        return self.__mul__(left);
        
        
        
class matrix1x4:
    def __init__(self, inputList=None):
        self.data = [0 for n in range(4)]
        if inputList is not None:
            for n in range(4):
                self.data[n] = inputList[n];
        
    def __repr__(self):
        returnString = "[ ";
        for n in range(3):
            returnString += str(self.data[n]) + ", "
        returnString += str(self.data[3]) + " ]"
        return returnString;
        
    def __mul__(self, right):
        if isinstance(right, matrix4x1):
            if isinstance(right.data[0], int) or isinstance(right.data[0], float):
                returnValueType = float
            elif hasattr(right.data[0].__class__, '__name__') and hasattr(right.data[0].__class__, '__dict__') and hasattr(right.data[0].__class__, '__bases__'):
                returnValueType = type(right.data[0].__class__.__name__, right.data[0].__class__.__bases__, right.data[0].__class__.__dict__)
            else:
                print "Data in right Matrix is unusable"
                return;
            returnValue = returnValueType()
            for k in range(4):
                returnValue += self.data[k] * right.data[k];
            return returnValue;
            
        elif isinstance(right, float) or isinstance(right, int):
            returnValue = [0 for x in range(4)];
            for n in range(4):
                returnValue[n] = self.data[n] * right;
            return matrix1x4(returnValue);
        
        elif isinstance(right, matrix4x4):
            returnValue = [0 for x in range(4)];
            for n in range(4):
                for k in range(4):
                    returnValue[n] += self.data[k] * right.data[k][n];
            return matrix1x4(returnValue);
            
        else:
            print "not correct type"
            return;
            
    def __rmul__(self, left):
        return self.__mul__(left);
            
            

class vector2:
    def __init__(self, xx=None, yy=None):
        self.x = 0 if xx is None else xx;
        self.y = 0 if yy is None else yy;
        
    def __repr__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")";
        
    def __add__(self, right):
        if isinstance(right, vector2):
            return vector2(self.x + right.x, self.y + right.y);
            
    def __sub__(self, right):
        if isinstance(right, vector2):
            return vector2(self.x - right.x, self.y - right.y);
        
    def __mul__(self, right):
        if isinstance(right, float) or isinstance(right, int):
            return vector2(self.x * right, self.y * right);
        else:
            print "Incorrect Type for multiplying vector2. Cannot multiply it with a " + str(type(other))
            return;
            
    def __rmul__(self, left):
        return self.__mul__(left);
            
            

def BezierInterpolate(curveName):
    rawKeyCount = mc.keyframe(curveName, query=True, keyframeCount=True)
    keyframes = mc.keyframe(curveName, query=True, timeChange=True, valueChange=True)
    
    if rawKeyCount < 4:
        print "Not enough control points, key count = " + str(rawKeyCount) + ", must have at least 4"
        return;
    
    keyCount = ((rawKeyCount - 4) // 3) * 3 + 4;
    curveCount = 1 + ((keyCount - 4) // 3);
    
    basisMatrix = matrix4x4([-1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0])
    
    for index in range(curveCount):
        p1KeyArrayIndex = 2 * (index * 3);
        p2KeyArrayIndex = 2 * (index * 3 + 1);
        p3KeyArrayIndex = 2 * (index * 3 + 2);
        p4KeyArrayIndex = 2 * (index * 3 + 3);
        p1 = vector2(keyframes[p1KeyArrayIndex], keyframes[p1KeyArrayIndex + 1]);
        p2 = vector2(keyframes[p2KeyArrayIndex], keyframes[p2KeyArrayIndex + 1]);
        p3 = vector2(keyframes[p3KeyArrayIndex], keyframes[p3KeyArrayIndex + 1]);
        p4 = vector2(keyframes[p4KeyArrayIndex], keyframes[p4KeyArrayIndex + 1]);

        startTime = int(keyframes[p1KeyArrayIndex])
        endTime = int(keyframes[p4KeyArrayIndex])
        timeSteps = abs(endTime - startTime)
        
        for t in range(timeSteps + 1):
            time = float(t) / timeSteps
            timeVector = matrix1x4([time ** 3, time ** 2, time, 1]);
            inputPointsVector = matrix4x1([p1, p2, p3, p4]);
            
            output = timeVector * basisMatrix * inputPointsVector;
            mc.setKeyframe(curveName, time=output.x, value=output.y)
            
            
            
def CatmullRomInterpolate(curveName):
    keyCount = mc.keyframe(curveName, query=True, keyframeCount=True)
    keyframes = mc.keyframe(curveName, query=True, timeChange=True, valueChange=True)
    
    basisMatrix = matrix4x4([-1, 3, -3, 1, 2, -5, 4, -1, -1, 0, 1, 0, 0, 2, 0, 0]);
    
    pointList = []
    for n in range(0, 2 * keyCount, 2):
        pointList.append(vector2(keyframes[n], keyframes[n + 1]));
        
    for curveIndex in range(0, keyCount - 1):
        p1 = pointList[(curveIndex - 1) if curveIndex != 0 else 0]
        p2 = pointList[curveIndex + 0]
        p3 = pointList[curveIndex + 1]
        p4 = pointList[(curveIndex + 2) if curveIndex != keyCount - 2 else curveIndex + 1]
        
        for t in range(10):
            time = float(t) / 10.0;
            timeVector = matrix1x4([time ** 3, time ** 2, time, 1]);
            
            inputPointsVector = matrix4x1([p1, p2, p3, p4]);
            
            output = timeVector * 0.5 * basisMatrix * inputPointsVector;
            mc.setKeyframe(curveName, time=output.x, value=output.y)
            
    
def BSplineInterpolate(curveName):
    keyCount = mc.keyframe(curveName, query=True, keyframeCount=True)
    keyframes = mc.keyframe(curveName, query=True, timeChange=True, valueChange=True)
    
    deBoorPoints = [];
    for n in range(2):
        deBoorPoints.append(vector2(keyframes[0], keyframes[1]))
    for index in range(0, len(keyframes), 2):
        deBoorPoints.append(vector2(keyframes[index], keyframes[index + 1]))
    for n in range(2):
        deBoorPoints.append(vector2(keyframes[-2], keyframes[-1]))
        
    basisMatrix = matrix4x4([-1, 3, -3, 1, 3, -6, 3, 0, -3, 0, 3, 0, 1, 4, 1, 0])
    
    for n in range(len(deBoorPoints) - 3):
        b1 = deBoorPoints[n]
        b2 = deBoorPoints[n + 1]
        b3 = deBoorPoints[n + 2]
        b4 = deBoorPoints[n + 3]
        
        timeSteps = 10
        
        for t in range(timeSteps + 1):
            time = float(t) / float(timeSteps)
            timeVector = matrix1x4([time ** 3, time ** 2, time, 1]);
            inputPointsVector = matrix4x1([b1, b2, b3, b4]);
            
            output = timeVector * (1.0 / 6.0) * basisMatrix * inputPointsVector;
            
            mc.setKeyframe(curveName, time=output.x, value=output.y)