import matplotlib.pyplot as plt
import csv
import numpy as np
from matplotlib import cm

"""Po metodi: https://github.com/CircusMonkey/covariance-ellipsoid"""

def get_cov_ellipsoid(cov, mu=np.zeros((3)), nstd=3):
    """
    Return the 3d points representing the covariance matrix
    cov centred at mu and scaled by the factor nstd.
    Plot on your favourite 3d axis. 
    Example 1:  ax.plot_wireframe(X,Y,Z,alpha=0.1)
    Example 2:  ax.plot_surface(X,Y,Z,alpha=0.1)
    """
    assert cov.shape==(3,3)

    # Find and sort eigenvalues to correspond to the covariance matrix
    eigvals, eigvecs = np.linalg.eigh(cov)
    eigvals[eigvals < 0] = 0
    idx = np.sum(cov,axis=0).argsort()
    eigvals_temp = eigvals[idx]
    idx = eigvals_temp.argsort()
    eigvals = eigvals[idx]
    eigvecs = eigvecs[:,idx]

    # Set of all spherical angles to draw our ellipsoid
    n_points = 100
    theta = np.linspace(0, 2*np.pi, n_points)
    phi = np.linspace(0, np.pi, n_points)

    # Width, height and depth of ellipsoid
    rx, ry, rz = nstd * np.sqrt(eigvals)

    # Get the xyz points for plotting
    # Cartesian coordinates that correspond to the spherical angles:
    X = rx * np.outer(np.cos(theta), np.sin(phi))
    Y = ry * np.outer(np.sin(theta), np.sin(phi))
    Z = rz * np.outer(np.ones_like(theta), np.cos(phi))

    # Rotate ellipsoid for off axis alignment
    old_shape = X.shape
    # Flatten to vectorise rotation
    X,Y,Z = X.flatten(), Y.flatten(), Z.flatten()
    X,Y,Z = np.matmul(eigvecs, np.array([X,Y,Z]))
    X,Y,Z = X.reshape(old_shape), Y.reshape(old_shape), Z.reshape(old_shape)
   
    # Add in offsets for the mean
    X = X + mu[0]
    Y = Y + mu[1]
    Z = Z + mu[2]
    
    return X,Y,Z


x = []
y = []
z = []

Ex = []
Ey = []
Ez = []

e11 = []
e12 = []
e13 = []

e21 = []
e22 = []
e23 = []

e31 = []
e32 = []
e33 = []

tmpx = []
tmpy = []
tmpz = []

mu =[]


with open('data.csv', 'r') as csvfile:
    datareader = csv.reader(csvfile,delimiter=',')
    for row in datareader:
        x.append(int(row[0]))
        y.append(int(row[1]))
        z.append(-int(row[2]))
        tmpx.append(int(float(row[3])))
        tmpx.append(int(float(row[4])))
        tmpx.append(int(float(row[5])))
        Ex.append(tmpx)
        tmpy.append(int(float(row[6])))
        tmpy.append(int(float(row[7])))
        tmpy.append(int(float(row[8])))
        Ey.append(tmpy)
        tmpz.append(int(float(row[9])))
        tmpz.append(int(float(row[10])))
        tmpz.append(int(float(row[11])))
        Ez.append(tmpz)
        tmpx = []
        tmpy = []
        tmpz = []

norm = plt.Normalize(-2998, 855)



# Setup the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.view_init(-90,-30)
ax.view_init(elev=44, azim=9)
plt.xlabel('x')
plt.ylabel('y')

for i in range(len(x)):

    #Za lepši prikaz odstranimo točki, ki so zelo oddaljene
    if abs(x[i]) > 4000:
        x[i] = 0
        y[i] = 0
        z[i] = 0

    if abs(y[i]) > 4000:
        x[i] = 0
        y[i] = 0
        z[i] = 0


    if abs(z[i]) > 3000:
        x[i] = 0
        y[i] = 0
        z[i] = 0

    if x[i] and y[i] and z[i] != 0:
        mu = [x[i],y[i],z[i]]
        cov = np.array([Ex[i],Ey[i],Ez[i]])

        s1 = np.random.multivariate_normal(mu, cov, (3))
        mu1_ = np.mean(s1, axis=0)
        cov1_ = np.cov(s1.T)

        colors = cm.winter(norm(z[i]))

        X1,Y1,Z1 = get_cov_ellipsoid(cov1_, mu1_, nstd=1)
        ax.plot_wireframe(X1,Y1,Z1, rstride=10, cstride=10, colors = colors, alpha = 0.1)

plt.show()