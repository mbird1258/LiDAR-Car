import time
import numpy as np
from server import get, send
from icp import icp
from dbscan import dbscan
from astar import astar
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sklearn.neighbors import KDTree

R = lambda theta: [[np.cos(theta), -np.sin(theta)],
                   [np.sin(theta),  np.cos(theta)]]

class MotorController:
    def __init__(self, WheelRadius, WheelGap, MaxRpm, MinRpm):
        '''
        m1: front left motor
        m2: front right motor
        m3: back left motor
        m4: back right motor
        
        angular velocity is in units of radians/second
        distance is in units of mm
        '''
        
        self.WheelRadius = WheelRadius
        self.WheelGap = WheelGap
        
        self.MaxRpm = MaxRpm
        self.MinRpm = MinRpm
        
        self.MaxForwardOffset = lambda RotationalOffset, time: np.clip((2*self.WheelRadius*time*self.MaxRpm-self.WheelGap*np.abs(RotationalOffset))/2, 0)
        self.MinForwardOffset = lambda RotationalOffset, time: np.clip((2*self.WheelRadius*time*self.MinRpm-self.WheelGap*np.abs(RotationalOffset))/2, 0)

    def ToMotorIn(self, ForwardOffset, RotationalOffset, time):
        w_l = (2*ForwardOffset+self.WheelGap*RotationalOffset)/(2*self.WheelRadius*time)
        w_r = (2*ForwardOffset-self.WheelGap*RotationalOffset)/(2*self.WheelRadius*time)
        
        return w_l, w_r, w_l, w_r

    def FromMotorIn(self, m1AngVel, m2AngVel, m3AngVel, m4AngVel, time):
        w_l = (m1AngVel + m3AngVel)/2
        w_r = (m2AngVel + m4AngVel)/2
        
        v_l = w_l*self.WheelRadius
        v_r = w_r*self.WheelRadius
        
        RotationalOffset = (v_l*time-v_r*time)/self.WheelRadius
        ForwardOffset = (v_l*time+v_r*time)/2
        
        return ForwardOffset, RotationalOffset

    def ToCartesianOffset(self, ForwardOffset, InitialRotation, RotationalOffset):
        MovementAngle = InitialRotation + RotationalOffset/2
        
        return ForwardOffset*np.cos(MovementAngle), ForwardOffset*np.sin(MovementAngle)

    def drive(self, waypoints):
        send(["Drive", waypoints])


class CarController:
    def __init__(self, 
                 WheelRadius, WheelGap, MaxRpm, MinRpm,
                 ProximityMatchThresh = 50, RayTraceThresh = 100, MovementDistThresh = 300, 
                 StartingConf = 2, MaxConf = 10, 
                 ClusterProximityThresh = 50, ClusterMinNeighbors = 2, ClusterMinPoints=5,
                 PathfindingTimeStep=0.1, PathfindingRadius=400, PathfindingThetaResolution=10, PathfindingDistanceResolution=6):
        self.FrameCount = 0
        self.ProximityMatchThresh = ProximityMatchThresh
        self.RayTraceThresh = RayTraceThresh
        self.MovementDistThresh = MovementDistThresh
        
        self.StartingConf = StartingConf
        self.MaxConf = MaxConf

        self.ClusterProximityThresh = ClusterProximityThresh
        self.ClusterMinNeighbors = ClusterMinNeighbors
        self.ClusterMinPoints = ClusterMinPoints

        self.PathfindingTimeStep = PathfindingTimeStep
        self.PathfindingRadius = PathfindingRadius
        self.PathfindingThetaResolution = PathfindingThetaResolution
        self.PathfindingDistanceResolution = PathfindingDistanceResolution
        
        self.map = np.empty((0, 2))
        self.MapConf = np.empty(0)
        
        self.LastPos = [0, 0, 0] # x, y, theta
        self.LastTime = time.time()
        
        self.waypoints = None
        self.MotorWaypoints = None

        self.MotorController = MotorController(WheelRadius, WheelGap, MaxRpm, MinRpm)

    def InterpolatePosition(self, CurrentTime):
        x, y, theta, time = self.waypoints.T
        m1, m2, m3, m4, _ = self.MotorWaypoints.T

        ind = np.searchsorted(time, CurrentTime, side='right')-1
        if ind == len(self.waypoints)-1:
            return x[-1], y[-1], theta[-1]
        
        if ind < 0:
            return x[0], y[0], theta[0]
        
        x, y, theta, time = x[ind], y[ind], theta[ind], time[ind]
        m1, m2, m3, m4 = m1[ind], m2[ind], m3[ind], m4[ind]
        dt = CurrentTime-time

        ForwardOffset, dtheta = self.MotorController.FromMotorIn(m1, m2, m3, m4, dt)
        dx, dy = self.MotorController.ToCartesianOffset(ForwardOffset, theta, dtheta)

        return x+dx, y+dy, theta+dtheta

    def GetLidar(self):
        send(["Get Lidar"])
        return get()[0]

    def PolarToCartesian(self, data):
        # FinalX, FinalY, FinalTheta = self.InterpolatePosition(data[-1][2])
        
        cartesian = []
        for angle, distance, time in data:
            radians = np.deg2rad(angle)
            x, y, theta = self.InterpolatePosition(time)

            cartesian.append([distance*np.cos(radians+theta)+x, distance*np.sin(radians+theta)+y])
            # cartesian.append([distance*np.cos(radians+theta-FinalTheta)+x-FinalX, distance*np.sin(radians+theta-FinalTheta)+y-FinalY])
        return np.array(cartesian)

    def LinePointDist(self, a, b, c, MinDist = 0):
        """
        a - point one on line
        b - point two on line
        c - point to find distance with
        """

        ab = b - a
        ac = c - a
        
        length = np.linalg.norm(ab)
        ClipVal = (length-MinDist)/length

        t = np.dot(ac, ab) / (length ** 2)
        t = np.clip(t, 0, ClipVal)

        ClosestPointOnLine = a + t * ab
        return np.linalg.norm(c - ClosestPointOnLine)

    def LinePointDistVectorised(self, a, b, c, MinDist = 0):
        a = np.array(a)
        b = np.array(b)
        c = np.array(c)
        
        ab = b - a[np.newaxis, :] # (m, 2)
        ac = c - a[np.newaxis, :] # (n,2)
        
        lengths = np.linalg.norm(ab, axis=1) # (m,)
        
        ClipVals = (lengths - MinDist) / lengths # (m,)
        
        t = np.einsum('ik,jk->ij', ab, ac) / (lengths ** 2)[:, np.newaxis] # (m, n)
        
        t = np.clip(t, 0, ClipVals[:, np.newaxis]) # (m, n)
        
        ClosestPointsOnLine = a + t[:, :, np.newaxis] * ab[:, np.newaxis, :] # (m, n, 2)
        
        distances = np.linalg.norm(c[np.newaxis, :, :] - ClosestPointsOnLine, axis=2) # (m, n)
        
        return distances

    def SplitPoints(self, OldPos, data):
        if not len(self.map):
            return np.empty((0, 2)), data, np.empty((0, 2))
        
        # Within proximity to last frame point -> preexisting point
        '''DistanceArray = np.sqrt(np.sum((data[:, np.newaxis, :]-self.map[np.newaxis, :, :])**2, axis=2))
        mask = np.any(DistanceArray < self.ProximityMatchThresh, axis=1)'''
        kdtree = KDTree(self.map)
        dists, _ = kdtree.query(data)
        mask = (dists < self.ProximityMatchThresh).flatten()
        
        PreexistingPoints = data[mask]
        data = data[~mask]
        
        # Has line of sight to last frame car position -> moving point
        DistanceArray = self.LinePointDistVectorised(OldPos[:-1], data, self.map) # np.array([[self.LinePointDist(OldPos[:-1], i, j) for j in self.map] for i in data])
        BlockingLineOfSightArray = DistanceArray < self.RayTraceThresh
        
        mask = np.any(BlockingLineOfSightArray, axis=1)
        
        MovingCloser = data[~mask]
        data = data[mask]
        BlockingLineOfSightArray = BlockingLineOfSightArray[mask]
        
        # If any points blocking line of sight to point within distance of point -> moving point
        # Else -> new point
        mask = np.zeros(data.shape[0]).astype(np.bool_)
        
        for ind, point in enumerate(data):
            BlockingPoints = self.map[BlockingLineOfSightArray[ind]]
            DistanceArr = np.hypot(BlockingPoints, point)
            
            if np.any(DistanceArr < self.MovementDistThresh):
                mask[ind] = True
        
        MovingAway = data[mask]
        NewPoints = data[~mask]
        
        return PreexistingPoints, NewPoints, np.append(MovingCloser, MovingAway, axis=0)

    def PreciseMatch(self, NewPos, PreexistingPoints, NewPoints, MovingPoints):
        if not len(PreexistingPoints):
            return NewPos, PreexistingPoints, NewPoints, MovingPoints
        
        (rot, trans), PrecisePoints = icp(self.map, PreexistingPoints)
        
        PrecisePos = rot @ NewPos[:-1] + trans
        NewAngle = rot @ np.array([np.cos(NewPos[-1]), np.sin(NewPos[-1])])
        NewAngle = np.arctan2(NewAngle[1], NewAngle[0])
        AdjustedNewPoints = (rot @ NewPoints.T + trans[:, np.newaxis]).T
        AdjustedMovingPoints = (rot @ MovingPoints.T + trans[:, np.newaxis]).T
        
        return PrecisePos+[NewAngle], PrecisePoints, AdjustedNewPoints, AdjustedMovingPoints

    def UpdateMap(self, NewPos, PreexistingPoints, NewPoints, MovingPoints):
        if not len(self.map):
            self.map = np.append(self.map, NewPoints, axis=0)
            self.MapConf = np.append(self.MapConf, [self.StartingConf for _ in NewPoints])
            return
        
        DistanceArray = np.sqrt(np.sum((PreexistingPoints[:, np.newaxis, :]-self.map[np.newaxis, :, :])**2, axis=2))
        WithinProximity = np.any(DistanceArray < self.ProximityMatchThresh, axis=0)
        # tree = KDTree(self.map)
        # dists, _ = tree.query(PreexistingPoints)
        
        # If within proximity to new point -> increase conf
        # WithinProximity = (dists < self.ProximityMatchThresh).flatten()
        self.MapConf += WithinProximity.astype(np.int8)
        
        # If not within proximity to new point and in line of sight -> decrease conf
        AllPoints = np.concatenate([PreexistingPoints, NewPoints, MovingPoints, self.map])
        DistanceArray = self.LinePointDistVectorised(NewPos[:-1], AllPoints, self.map) # [[self.LinePointDist(NewPos[:-1], i, j, MinDist=10) for j in self.map] for i in AllPoints]
        InLineOfSightArray = np.all(DistanceArray > self.RayTraceThresh, axis=0)
        self.MapConf -= np.all([~WithinProximity, InLineOfSightArray], axis=0).astype(np.int8)
        
        # Cull bad points
        mask = self.MapConf >= 0
        self.MapConf = self.MapConf[mask]
        self.map = self.map[mask]

        # Add new points to graph
        self.map = np.append(self.map, NewPoints, axis=0)
        self.MapConf = np.append(self.MapConf, [self.StartingConf for _ in NewPoints])

    def GetMotion(self, MovingPoints, time):
        if not len(MovingPoints):
            return [], [], [], np.empty((0, 2))
        
        # DBSCAN to cluster points
        clusters = dbscan(MovingPoints, self.ClusterProximityThresh, self.ClusterMinNeighbors)
        
        # Exclude any cluster under a threshold of points
        clusters = [cluster for cluster in clusters if len(cluster) >= self.ClusterMinPoints]

        VelArr = []
        AngVelArr = []
        InitialAngleArr = []
        for cluster in clusters:
            # Find center of cluster to normalise map and cluster before perfroming icp so that the returned R value is around the center of the cluster, not (0,0)
            CenterOfCluster = np.mean(cluster, axis=0)

            # ICP to find offset from last frame in R, T for each cluster (in which ICP rotates around the CoM of cluster)
            (rot, trans), _ = icp(self.map - CenterOfCluster, cluster - CenterOfCluster)

            AngVel = np.arctan2(rot[1,0], rot[0,0])/time
            vel = np.sqrt(np.sum(trans**2))/time
            InitialAngle = np.arctan2(trans[1], trans[0])

            # Use T and R with time offset to calculate velocity and angular velocity of cluster
            VelArr.append(vel)
            AngVelArr.append(AngVel)
            InitialAngleArr.append(InitialAngle)
        
        return VelArr, AngVelArr, InitialAngleArr, clusters

    def NextNodes(self, node, map, MobileMap, MinForwardOffsetFunc, MaxForwardOffsetFunc, TimeStep, CarRadius, ThetaResolution, DistanceResolution):
        x1, y1, theta1, t = node

        angles = np.linspace(0, 2*np.pi, ThetaResolution, endpoint=False)
        MaxDistArr = MaxForwardOffsetFunc(angles, TimeStep)
        MinDistArr = MinForwardOffsetFunc(angles, TimeStep)
        
        angles = np.repeat(angles, DistanceResolution)
        dists = np.concatenate([np.linspace(MinDist, MaxDist, DistanceResolution) for MinDist, MaxDist in zip(MinDistArr, MaxDistArr)])
        dx, dy = self.MotorController.ToCartesianOffset(dists, theta1, angles)
        x, y = x1+dx, y1+dy

        obstacles = np.append(map, MobileMap(t), axis=0)
        kdtree = KDTree(obstacles)

        dists, _ = kdtree.query(np.array([x, y]).T).flatten()
        mask = dists > CarRadius
        neighbors = np.array([x[mask], y[mask], (theta1+angles[mask])%(2*np.pi), np.full(np.sum(mask), t+TimeStep)]).T
        
        return neighbors

        '''x1, y1, theta1, t = node
        neighbors = []
        for i in range(ThetaResolution):
            dtheta = 2*np.pi*i/ThetaResolution
            
            MaxDist = MaxForwardOffsetFunc(dtheta, TimeStep)
            MinDist = MinForwardOffsetFunc(dtheta, TimeStep)

            for dist in np.linspace(MinDist, MaxDist, DistanceResolution):
                dx, dy = self.MotorController.ToCartesianOffset(dist, theta1, dtheta)

                if min([self.LinePointDist([x1, y1], [x1+dx, y1+dy], obstacle)] for obstacle in obstacles) >= CarRadius:
                    neighbors.append([x1+dx, y1+dy, theta1+dtheta, t+TimeStep])

        return neighbors'''

    def GetPath(self, x, y, theta, goal):
        waypoints = astar((x, y, theta, 0), goal, self.NextNodes, (self.map, self.MobileMap, self.MotorController.MinForwardOffset, self.MotorController.MaxForwardOffset, self.PathfindingTimeStep, self.PathfindingRadius, self.PathfindingThetaResolution, self.PathfindingDistanceResolution))
        waypoints[:, 3] += time.time()

        return waypoints

    def WaypointsToMotorWaypoints(self, waypoints):
        # Convert waypoints to a list of angular velocities for each motor
        x, y, theta, time = waypoints.T
        dx = x[1:]-x[:-1]
        dy = y[1:]-y[:-1]
        dtheta = theta[1:]-theta[:-1]
        dt = time[1:]-time[:-1]
        ForwardOffset = np.hypot(dx, dy)

        m1AngVel, m2AngVel, m3AngVel, m4AngVel = self.MotorController.ToMotorIn(ForwardOffset, dtheta, dt)
        return np.array([m1AngVel, m2AngVel, m3AngVel, m4AngVel, dt]).T

    def GetMobileMapAsFunc(self, VelArr, AngVelArr, InitialAngleArr, clusters):
        if not VelArr:
            return lambda time: np.empty((0, 2))
        
        def func(time):
            MovedClusters = []

            for vel, AngVel, InitialAngle, cluster in zip(VelArr, AngVelArr, InitialAngleArr, clusters):
                distance, theta, dtheta = vel*time, InitialAngle, AngVel*time
                dx, dy = self.MotorController.ToCartesianOffset(distance, theta, dtheta)
                
                CenterOfCluster = np.mean(cluster, axis=0, keepdims=True)

                MovedClusters.append(R(theta) @ (cluster - CenterOfCluster) + np.array([[dx, dy]]) + CenterOfCluster)

            return np.concatenate(MovedClusters, axis=0)

        return func

    # def main(self, goal, debug=False):
    #     LidarOut = self.GetLidar()
    #     if not LidarOut:
    #         return
    #     LidarCart = self.PolarToCartesian(LidarOut)
        
    #     t = time.time()
    #     pos = self.InterpolatePosition(t) if isinstance(self.waypoints, np.ndarray) else (0, 0, 0)
    #     preexisting, new, moving = self.SplitPoints(self.LastPos, LidarCart)
    #     PrecisePos, PrecisePreexisting, PreciseNew, PreciseMoving = self.PreciseMatch(pos, preexisting, new, moving)
    #     self.UpdateMap(PrecisePos, PrecisePreexisting, PreciseNew, PreciseMoving)
    #     VelArr, AngVelArr, InitialAngleArr, clusters = self.GetMotion(PreciseMoving, time-self.LastTime)
    #     self.MobileMap = self.GetMobileMapAsFunc(VelArr, AngVelArr, InitialAngleArr, clusters)
        
    #     self.waypoints = self.GetPath(*PrecisePos, goal)
    #     self.MotorWaypoints = self.WaypointsToMotorWaypoints(self.waypoints)
        
    #     self.MotorController.drive(self.MotorWaypoints)
        
    #     if debug:
    #         self.DebugGraphs(pos, PrecisePos, preexisting, new, PrecisePreexisting, PreciseNew)
        
    #     self.LastPos = pos

    def main(self, debug=False, DebugVideo=False):
        LidarOut = self.GetLidar()
        if not len(LidarOut):
            return False
        LidarCart = self.PolarToCartesian(LidarOut)
        
        t = time.time()
        pos = self.InterpolatePosition(t) if isinstance(self.waypoints, np.ndarray) else (0, 0, 0)
        preexisting, new, moving = self.SplitPoints(self.LastPos, LidarCart)
        PrecisePos, PrecisePreexisting, PreciseNew, PreciseMoving = self.PreciseMatch(pos, preexisting, new, moving)
        self.UpdateMap(PrecisePos, PrecisePreexisting, PreciseNew, PreciseMoving)
        
        if debug:
            self.DebugGraphs(pos, PrecisePos, preexisting, new, PrecisePreexisting, PreciseNew)
        
        if DebugVideo:
            self.DebugVideo(pos, PrecisePos, preexisting, new, PrecisePreexisting, PreciseNew)
        
        self.LastPos = pos
        
        return True

    def DebugGraphs(self, pos, PrecisePos, preexisting, new, PrecisePreexisting, PreciseNew, dir='debug'):
        try:
            os.makedirs(dir, exist_ok=True)
            # Plot 1: Interpolate waypoints curve, waypoints, position, and agent map
            fig1, ax1 = plt.subplots()
            ax1.plot(*np.array([self.InterpolatePosition(t)[:-1] for t in np.linspace(self.waypoints[0][3], self.waypoints[-1][3], 1000)]).T)
            ax1.scatter(self.waypoints[:, 0], self.waypoints[:, 1], c='b', marker='x', label='Waypoints')
            ax1.scatter(self.map[:, 0], self.map[:, 1], c='grey', alpha=0.2, label='Map')
            ax1.scatter(preexisting[:, 0], preexisting[:, 1], c='green', alpha=0.8, label='Scan')
            ax1.scatter(new[:, 0], new[:, 1], c='red', alpha=0.8, label='Scan')
            ax1.scatter(PrecisePos[0], PrecisePos[1], c='black', label='Current Position')
            
            ax1.legend(loc='upper right')
            ax1.set_title("Position, Waypoints, and Map")
            fig1.savefig(os.path.join(dir, '1_position_waypoints_map' + str(time.time()) + '.png'))
            plt.close(fig1)
            
            # Plot 2: Agent positions, scan points with line of sights
            fig2, ax2 = plt.subplots()
            ax2.scatter(self.LastPos[0], self.LastPos[1], c='grey', marker='x', s=100, label='Old Position')
            ax2.scatter(pos[0], pos[1], c='black', marker='x', s=100, label='Current Position')
            ax2.scatter(preexisting[:, 0], preexisting[:, 1], c='red', label='Preexisting')
            ax2.scatter(new[:, 0], new[:, 1], c='green', label='New')
            # Draw line of sights
            for point in preexisting:
                ax2.plot([self.LastPos[0], point[0]], [self.LastPos[1], point[1]], 'g-', alpha=0.2, lw=0.5)
            for point in new:
                ax2.plot([self.LastPos[0], point[0]], [self.LastPos[1], point[1]], 'r-', alpha=0.2, lw=0.5)
            
            ax2.legend(loc='upper right')
            ax2.set_title("Current Scan with Line of Sights")
            fig2.savefig(os.path.join(dir, '2_scan_lines' + str(time.time()) + '.png'))
            plt.close(fig2)
            
            # Plot 3: Agent map before and after precise adjustment
            fig3, ax3 = plt.subplots()
            ax3.scatter(self.map[:, 0], self.map[:, 1], c='grey', alpha=0.3, label='Map')
            ax3.scatter(preexisting[:, 0], preexisting[:, 1], c='red', s=20, label='Preexisting')
            ax3.scatter(new[:, 0], new[:, 1], c='green', s=20, label='New')
            
            ax3.legend(loc='upper right')
            ax3.set_title("Map Before ICP Adjustment")
            fig3.savefig(os.path.join(dir, '3_map_before' + str(time.time()) + '.png'))
            plt.close(fig3)
            
            fig3, ax3 = plt.subplots()
            ax3.scatter(self.map[:, 0], self.map[:, 1], c='grey', alpha=0.3, label='Map')
            ax3.scatter(PrecisePreexisting[:, 0], PrecisePreexisting[:, 1], c='red', s=20, label='Adjusted Preexisting')
            ax3.scatter(PreciseNew[:, 0], PreciseNew[:, 1], c='green', s=20, label='Adjusted New')
            
            ax3.legend(loc='upper right')
            ax3.set_title("Map After ICP Adjustment")
            fig3.savefig(os.path.join(dir, '3_map_adjusted' + str(time.time()) + '.png'))
            plt.close(fig3)
            
            # Plot 4: Agent map with confidence values
            if len(self.map) > 0:
                fig4, ax4 = plt.subplots()
                colors = 100*self.MapConf/self.MaxConf
                sc = ax4.scatter(self.map[:, 0], self.map[:, 1], cmap='viridis', c=colors)
                
                cbar = plt.colorbar(sc, ax=ax4)
                cbar.set_label('Confidence')
                ax4.set_title("Map with Confidence Values")
                fig4.savefig(os.path.join(dir, '4_confidence_map' + str(time.time()) + '.png'))
                plt.close(fig4)
            
            return
            
            # Plot 5: Moving map animation
            fig5, ax5 = plt.subplots()
            
            def animate(frame):
                t = frame * 0.2  # Time step per frame
                ax5.clear()
                ax5.scatter(self.map[:, 0], self.map[:, 1], c='grey', alpha=0.3, label='Static Map')
                
                mobile = self.MobileMap(t)
                ax5.scatter(mobile[:, 0], mobile[:, 1], c='blue', alpha=0.5, label='Moving Map')
                
                ax5.set_xlim(np.min(self.map[:, 0])-100, np.max(self.map[:, 0])+100)
                ax5.set_ylim(np.min(self.map[:, 1])-100, np.max(self.map[:, 1])+100)
                ax5.set_title(f"Mobile Map at t={t:.1f}s")
                ax5.legend(loc='upper right')
            
            ani = animation.FuncAnimation(fig5, animate, frames=20, interval=200)
            ani.save(os.path.join(dir, '5_mobile_map.gif'), writer='pillow', fps=5)
            plt.close(fig5)
            
            plt.close('all')
        
        except Exception as e:
            print(e)
            return

    def DebugVideo(self, pos, PrecisePos, preexisting, new, PrecisePreexisting, PreciseNew, dir='debug'):
        os.makedirs(dir, exist_ok=True)
        
        # Plot 2: Agent positions, scan points with line of sights
        fig, ax = plt.subplots()
        ax.scatter(self.map[:, 0], self.map[:, 1], c='grey', alpha=0.2, label='Map')
        ax.scatter(self.LastPos[0], self.LastPos[1], c='grey', marker='x', s=100, label='Old Position')
        ax.scatter(pos[0], pos[1], c='black', marker='x', s=100, label='Current Position')
        ax.scatter(preexisting[:, 0], preexisting[:, 1], c='red', label='Preexisting')
        ax.scatter(new[:, 0], new[:, 1], c='green', label='New')
        for point in preexisting:
            ax.plot([self.LastPos[0], point[0]], [self.LastPos[1], point[1]], 'g-', alpha=0.2, lw=0.5)
        for point in new:
            ax.plot([self.LastPos[0], point[0]], [self.LastPos[1], point[1]], 'r-', alpha=0.2, lw=0.5)
        ax.set_xlim([-1000, 4000])
        ax.set_ylim([-1000, 4000])
        ax.legend(loc='upper right')
        ax.set_title("Splitting Points")
        fig.savefig(os.path.join(dir, f'{self.FrameCount}.png'))
        plt.close(fig)
        self.FrameCount += 1
        
        # Plot 1: Waypoints and map
        fig, ax = plt.subplots()
        ax.plot(*np.array([self.InterpolatePosition(t)[:-1] for t in np.linspace(self.waypoints[0][3], self.waypoints[-1][3], 1000)]).T)
        ax.scatter(self.waypoints[:, 0], self.waypoints[:, 1], c='b', marker='x', label='Waypoints')
        ax.scatter(self.map[:, 0], self.map[:, 1], c='grey', alpha=0.2, label='Map')
        ax.scatter(preexisting[:, 0], preexisting[:, 1], c='red', label='Preexisting')
        ax.scatter(new[:, 0], new[:, 1], c='green', label='New')
        ax.scatter(pos[0], pos[1], c='black', label='Current Position')
        ax.set_xlim([-1000, 4000])
        ax.set_ylim([-1000, 4000])
        ax.legend(loc='upper right')
        ax.set_title("Before Adjustment")
        fig.savefig(os.path.join(dir, f'{self.FrameCount}.png'))
        plt.close(fig)
        self.FrameCount += 1
        
        # Plot 3: Map after adjustment
        fig, ax = plt.subplots()
        ax.plot(*np.array([self.InterpolatePosition(t)[:-1] for t in np.linspace(self.waypoints[0][3], self.waypoints[-1][3], 1000)]).T)
        ax.scatter(self.waypoints[:, 0], self.waypoints[:, 1], c='b', marker='x', label='Waypoints')
        ax.scatter(self.map[:, 0], self.map[:, 1], c='grey', alpha=0.2, label='Map')
        ax.scatter(PrecisePreexisting[:, 0], PrecisePreexisting[:, 1], c='red', label='Preexisting')
        ax.scatter(PreciseNew[:, 0], PreciseNew[:, 1], c='green', label='New')
        ax.scatter(PrecisePos[0], PrecisePos[1], c='black', label='Current Position')
        ax.set_xlim([-1000, 4000])
        ax.set_ylim([-1000, 4000])
        ax.legend(loc='upper right')
        ax.set_title("After Adjustment")
        fig.savefig(os.path.join(dir, f'{self.FrameCount}.png'))
        plt.close(fig)
        self.FrameCount += 1