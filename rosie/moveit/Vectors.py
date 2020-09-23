class V4D:

    def __init__(self, x, y, z, w):
        self.__x = x;
        self.__y = y;
        self.__z = z;
        self.__w = w;

    def __add__(self, vector):
        x = self.__x + vector.__x;
        y = self.__y + vector.__y;
        z = self.__z + vector.__z;
        w = self.__w + vector.__w;
        return V4D(x, y, z, w);

    def __sub__(self, vector):
        x = self.__x - vector.__x;
        y = self.__y - vector.__y;
        z = self.__z - vector.__z;
        w = self.__w - vector.__w;
        return V4D(x, y, z, w);

    def __eq__(self, vector):
        if self.__x == vector.__x:
            if self.__y == vector.__y:
                if self.__z == vector.__z:
                    if self.__w == vector.__w:
                        return True;
        return False;

    def __gt__(self, vector):
        if self.__x > vector.__x:
            if self.__y > vector.__y:
                if self.__z > vector.__z:
                    return True;
        return False;

    def display(self):
        print "(",  self.__x, ", ", self.__y, ", ", self.__z, ", ", self.__w, ")";

    def x(self):
        return self.__x;
    def y(self):
        return self.__y;
    def z(self):
        return self.__z;
    def w(self):
        return self.__w;
