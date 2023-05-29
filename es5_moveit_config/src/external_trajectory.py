'Release date: 23-05-14 12:32:33\xa0\n'
from es_master.srv import moveJointTrajectory, moveJointTrajectoryResponse
import rospy
import robotLib.es5 as es
es.init()
var = es.Shared_variables()
wld = es.Shared_variables('es_welding_lists')
pts = es.Points()
def callback(request):
    for i in range(len(request.trajectory.points)):
        p = es.Position([], request.trajectory.points[i].positions, 'default', 'default')
        es.move(es.Point, end=p, time=request.trajectory.points[i].time_from_start.secs)
        
    es.execute_move()
    response = moveJointTrajectoryResponse(True, 'trajectory finished')
    return response
    
def main():
    service = rospy.Service('/external_trajectory_service', moveJointTrajectory, callback)
    i = 0
    while True:
        pass
        
    
if (__name__ == '__main__'):
    main()
