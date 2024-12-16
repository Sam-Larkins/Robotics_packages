from collections import namedtuple

# Points should be 2-tuple (x,y)
Point = namedtuple("Point", ("x", "y"))
OrderedEdge = namedtuple("OrderedEdge", ("lp", "rp"))


class PointList(list):
    bounds = None

    def __init__(self, p_in):
        p_out = []
        for p in p_in:
            if not isinstance(p, Point):
                assert len(p) == 2
                p = Point(p[0], p[1])
            p_out.append(p)
        super(PointList, self).__init__(p_out)

    def min_yx_index(self):
        im = 0
        for i, pi in enumerate(self):
            if pi.y < self[im].y:
                im = i
            elif pi.y == self[im].y and pi.x < self[im].x:
                im = i
        return im

    def swap(self, i, j):
        self[i], self[j] = self[j], self[i]

    def get_bounds(self):
        # returns [minx, maxx, miny, maxy]
        if self.bounds is None:
            self.bounds = [
                min(self, key=lambda t: t[0])[0],
                max(self, key=lambda t: t[0])[0],
                min(self, key=lambda t: t[1])[1],
                max(self, key=lambda t: t[1])[1],
            ]
        return self.bounds

    def get_xy(self):
        x, y = zip(*self)
        return x, y


class Polygon(PointList):
    def edges(self):
        for i in range(len(self) - 1):
            yield self[i], self[i + 1]
        yield self[-1], self[0]

    def get_edge(self, i):
        return self[i], self[(i + 1) % len(self)]

    def point_inside_cn(self, p):
        #  crossing_number_poly(): crossing number test for a point in a polygon
        #       Input:   P = a point,
        #                V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
        #       Return:  0 = outside, 1 = inside
        #  This code is patterned after [Franklin, 2000] (normally just use winding method)
        cn = 0  # crossing number counter

        #  loop through all edges of the polygon
        for v0, v1 in self.edges():  # edge from V[i]  to V[i+1]
            if ((v0.y <= p.y) and (v1.y > p.y)) or (
                (v0.y > p.y) and (v1.y <= p.y)
            ):  # a downward crossing
                # Compute  the actual edge-ray intersect x-coordinate
                vt = (p.y - v0.y) / (v1.y - v0.y)
                if p.x < v0.x + vt * (v1.x - v0.x):  # P.x < intersect
                    cn += 1  # a valid crossing of y=P.y right of P.x

        return bool(cn % 2)  # 0 if even (out), and 1 if  odd (in)

    def point_inside(self, p):
        #  point_inside(): winding number test for a point in a polygon
        #       Input:   p = a point,
        #       Return:  wn = the winding number (=0 only when P is outside)

        wn = 0  # the  winding number counter

        #  loop through all edges of the polygon
        for v0, v1 in self.edges():  # edge from V[i] to  V[i+1]
            if v0.y <= p.y:  # start y <= P.y
                if v1.y > p.y:  # an upward crossing
                    if is_left(v0, v1, p) > 0:  # P left of  edge
                        wn += 1  # have  a valid up intersect

            else:  # start y > P.y (no test needed)
                if v1.y <= p.y:  # a downward crossing
                    if is_left(v0, v1, p) < 0:  # P right of  edge
                        wn -= 1  # have  a valid down intersect
        return wn

    def intersect(self, poly2):
        assert isinstance(poly2, Polygon)
        bounds1 = self.get_bounds()
        bounds2 = poly2.get_bounds()

        if (
            bounds2[1] <= bounds1[0]
            or bounds2[0] >= bounds1[1]
            or bounds2[3] <= bounds1[2]
            or bounds2[2] >= bounds1[3]
        ):
            return False

        for p in poly2:
            if self.point_inside(p):
                return True
        for p in self:
            if poly2.point_inside(p):
                return True

        all_edges = []

        def add_ordered_edges(edge_list, new_edges):
            for p0, p1 in new_edges:
                if p0 < p1:
                    edge_list.append(OrderedEdge(p0, p1))
                else:
                    edge_list.append(OrderedEdge(p1, p0))

        my_edges = []
        add_ordered_edges(my_edges, self.edges())
        your_edges = []
        add_ordered_edges(your_edges, poly2.edges())
        for e1 in my_edges:
            for e2 in your_edges:
                if line_intersect(e1, e2):
                    return True
        return False


class Rectangle(Polygon):
    def __init__(self, xlim, ylim):
        super(Rectangle, self).__init__(
            [
                [xlim[0], ylim[0]],
                [xlim[1], ylim[0]],
                [xlim[1], ylim[1]],
                [xlim[0], ylim[1]],
            ]
        )


def line_intersect(l0, l1):
    # Assume ordered lines (OrderedEdge objects)
    lsign = is_left(l0.lp, l0.rp, l1.lp)  #  l1 left point sign
    rsign = is_left(l0.lp, l0.rp, l1.rp)  #  l1 right point sign
    if lsign * rsign >= 0:  # l1 endpoints have same sign  relative to l0
        return False  # => on same side => no intersect is possible
    lsign = is_left(l1.lp, l1.rp, l0.lp)  #  l0 left point sign
    rsign = is_left(l1.lp, l1.rp, l0.rp)  #  l0 right point sign
    if lsign * rsign >= 0:  # l0 endpoints have same sign  relative to l1
        return False  # => on same side => no intersect is possible
    return True  # => an intersect exists


def is_left(p0, p1, p2):
    # tests if point P2 is Left|On|Right of the line P0 to P1.
    # returns: >0 for left, 0 for on, and <0 for  right of the line.
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)
