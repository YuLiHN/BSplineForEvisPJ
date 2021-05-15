import numpy as np
import matplotlib.pyplot as plt
import scipy.io

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
        if t[i]<=u<=t[i+1]:
            return 1
        else:
            return 0
    else:
        if ((t[i+p]-t[i])<1e-5):
            w1 = 0
        else:
            w1 = (u-t[i]) / (t[i+p]-t[i])
        if ((t[i+p+1] - t[i+1])<1e-5):
            w2 = 0
        else:
            w2 = (t[i+p+1]-u) / (t[i+p+1] - t[i+1])
        return basicFun(t,i,p-1,u) * w1 + basicFun(t,i+1,p-1,u) * w2

def deBoor(x, t, c, p):
    """Evaluates S(x).

    Arguments
    ---------
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
    res = np.zeros((m,4))
    for p in range(4):
        for p_ in range(4-p):
            for k in range(m):
                res[k,p_] = basicFun(t, p_, p, u[k])

        plt.figure()
        plt.title(f'B-spline basic Function of {p}th degree')
        plt.plot(u,res[:,:4-p])
        plt.show()

def plotCumulativeBasicFun():
    m = 100
    t = np.array([0,.25,.5,.75,1.])
    u = np.linspace(0.,1.,m)
    res = np.zeros((m,4))
    for p in range(4):
        for p_ in range(4-p):
            for k in range(m):
                res[k,p_] = basicFun(t, p_, p, u[k])

        plt.figure()
        plt.title(f'B-spline basic Function of {p}th degree')
        plt.plot(u,res[:,:4-p])
        plt.show()
        if (p == 1) :
            plt.figure()
            plt.title(f'B-spline cumulative basic Function of {p}th degree')
            plt.plot(u[:-25],res[:-25,0]+res[:-25,1]+res[:-25,2])
            plt.plot(u[:-25],res[:-25,1]+res[:-25,2])
            plt.plot(u[:-25],res[:-25,2])
            plt.show()

def plotBSpline(t,c):
    
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

def ploReverseBSpline():
    k=3
    n=5

    file = scipy.io.loadmat('interp.mat')
    X = file['X']
    data = X[::25,:]

    ls = np.sqrt((data[1:,0]-data[:-1,0])**2 + (data[1:,1]-data[:-1,1])**2)
    L = ls.sum()
    knots = np.concatenate((np.zeros(4),np.array([ls[0],ls[:2].sum(),ls[:3].sum()])/L, np.ones(4)))

    delta = knots[1:] - knots[:-1]
    a = np.zeros(4)
    b = np.zeros(4)
    c = np.zeros(4)
    e = np.zeros((4,2))
    for i in range(1,4):
        a[i] = (delta[i+2])**2 / (delta[i]+delta[i+1]+delta[i+2])
        b[i] = delta[i+2]*(delta[i]+delta[i+1]) / (delta[i]+delta[i+1]+delta[i+2]) + delta[i+1]*(delta[i+2]+delta[i+3]) / (delta[i+3]+delta[i+1]+delta[i+2])
        c[i] = (delta[i+1])**2 / (delta[i+3]+delta[i+1]+delta[i+2])
        e[i,:] = (delta[i+1]+delta[i+2]) * data[i-1,:]

    A = np.zeros((4,4))
    a1 = 1- delta[3]*delta[4] / (delta[3]+delta[4])**2
    b1 = delta[3] /(delta[3]+delta[4]) * (delta[3] /(delta[3]+delta[4]) - delta[3] /(delta[3]+delta[4]+delta[5]))
    c1 = delta[3]**2 / ((delta[3]+delta[4])*(delta[3]+delta[4]+delta[5]))
    e1 = 1/3 * (data[0,:] +2*data[1,:])

    an = -delta[n]**2/((delta[n-1]+delta[n])*(delta[n-1]+delta[n]+delta[n-2]))
    bn = delta[n]/(delta[n-1]+delta[n]) * (delta[n]/(delta[n-2]+delta[n-1]+delta[n]) - delta[n-1]/(delta[n-1]+delta[n]))
    cn = delta[n-1]*delta[n]/(delta[n-1]+delta[n])**2 - 1
    en = -1/3 * (data[-1,:] + 2*data[-2,:])
    # an = -delta[n]**2/((delta[n-1]+delta[n])*(delta[n-1]+delta[n]+delta[n-2]))
    # bn = delta[n]/(delta[n-1]+delta[n]) * (delta[n]/(delta[n-2]+delta[n-1]+delta[n]) - delta[n-1]/(delta[n-1]+delta[n]))
    # cn = delta[n-1]*delta[n]/(delta[n-1]+delta[n])**2 - 2
    # en = -data[-1,:] - data[-2,:]

    A = np.zeros((4,4))
    A[0,0] = a1
    A[0,1] = b1
    A[0,2] = c1

    A[1,0] = a[2]
    A[1,1] = b[2]
    A[1,2] = c[2]

    A[2,1] = a[3]
    A[2,2] = b[3]
    A[2,3] = c[3]

    A[3,1] = an
    A[3,2] = bn
    A[3,3] = cn
    print("A\n",A)

    e[0,:] = e1
    e[1,:] = e[2]
    e[2,:] = e[3]
    e[3,:] = en

    print("e\n",e)

    d = np.linalg.inv(A) @ e
    # d[-1,1] = 2.5

    d = np.vstack((data[0,:],data[0,:],d,data[-1,:]))
    print("d\n",d)

    m = 101
    u = np.linspace(0,1,m)
    res = np.ones((m,2))
    for i in range(m):
        res[i,:] = deBoor(u[i], knots, d, 3)


    plt.figure()
    plt.scatter(data[:,0],data[:,1],label = "data points")
    plt.scatter(d[:,0], d[:,1],label = "recovered control points")
    plt.plot(X[:,0],X[:,1],label = "original B-spline trajectory")
    plt.plot(res[:,0],res[:,1], label = "recovered B-spline trajectory")
    plt.legend()
    plt.show()
    # plotBSpline(knots, d)





if __name__ == "__main__":
    t = np.array([0,0,0,0,0.25,0.5,0.75,1.,1.,1.,1.])
    c = np.array([[1,3],[2,1],[3,6],[4,4],[5,6],[6,4],[7,9]]) 

    #plotBasicFun()
    plotCumulativeBasicFun()
    plotBSpline(t,c)
    #ploReverseBSpline()
