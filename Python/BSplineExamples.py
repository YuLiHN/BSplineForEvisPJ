import numpy as np
import matplotlib.pyplot as plt

def basicFun(t,i,p,u):

    """Calculates the basic function. Recursive form of b-spline(defination).

    Arguments
    ---------
    t: Array of knot positions
    i: ith knot interval
    p: degree of B-spline
    u: current position
    """

    if p==0:
        if t[i]<u<t[i+1]:
            return 1
        else:
            return 0
    else:
        return basicFun(t,i,p-1,u) * (u-t[i]) / (t[i+p]-t[i]) + basicFun(t,i+1,p-1,u) * (t[i+p+1]-u) / (t[i+p+1] - t[i+1])

def deBoor(x, t, c, p):
    """Evaluates S(x).

    Arguments
    ---------
    k: Index of knot interval that contains x.
    x: Position.
    t: Array of knot positions, needs to be padded.
    c: Array of control points.
    p: Degree of B-spline.
    """
    
    #automatically find in which interval k lies
    k = -1
    for i in range(np.shape(t)[0]):
        if x < t[i]:
            k = i - 1
            break
    if k == -1:
        return c[-1,:]
    
    d = [c[j + k - p,:] for j in range(0, p + 1)]

    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            alpha = (x - t[j + k - p]) / (t[j + 1 + k - r] - t[j + k - p])
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]

    return d[p]

def plotBasicFun():
    m = 100
    t = np.array([0,.25,.5,.75,1.])
    u = np.linspace(0.,1.,m)
    res = np.zeros(m)
    for p in range(4):
        for k in range(m):
            res[k] = basicFun(t, 0, p, u[k])

        plt.figure()
        plt.title(f'B-spline basic Function of {p}th degree')
        plt.plot(u,res)
        plt.show()

def plotBSpline():
    t = np.array([0,0,0,0,0.25,0.5,0.75,1.,1.,1.,1.])
    c = np.array([[1,3],[2,1],[3,6],[4,4],[5,6],[6,4],[7,9]]) 
    p = 3 

    m = 101
    u = np.linspace(0,1,m)
    res = np.ones((m,2))
    for i in range(m):
        res[i,:] = deBoor(u[i], t, c, p)

    plt.figure()
    plt.plot(c[:,0], c[:,1], 'r.-',label = "control points")
    plt.plot(res[:,0],res[:,1], label = "B-spline trajectory")
    plt.title("plot of B-spline using De Boor Algorithm")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    plotBasicFun()
    plotBSpline()