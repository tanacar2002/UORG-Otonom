import asyncio
import math as m
from mavsdk import System
from mavsdk import mission_raw as mr
import RPi.GPIO as GPIO
""" N
 90 | 0
W---|---E
 180|270
""" 
angle = (360-342) * m.pi / 180 #Angle from North to West
pts1 =[(0,0,2)
        ,(0,0,7)
        ,(0,5,7)
        ,(0,5,2)
        ,(0,0,2)
        ,(5,0,2)
        ,(5,5,2)
        ,(0,5,2)
        ,(0,5,7)
        ,(5,5,7)
        ,(5,5,2)
        ,(5,0,2)
        ,(5,0,7)
        ,(5,5,7)
        ,(5,0,7)
        ,(0,0,7)
        ,(0,0,5)]
PIN = 12
sroi = (15/4,5*m.sqrt(3)/12,5)#(5/2,5*m.sqrt(3)/6,5)
rois = [(15/4,5*m.sqrt(3)/12,5)
        ,(5/4,5*m.sqrt(3)/12,5)
        ,(5/2,5*m.sqrt(3)/3,5)]
triangle = [(0,0,5)
        ,(5/2,5*m.sqrt(3)/2,5)
        ,(5,0,5)]
pts2 =[*triangle,*triangle,*triangle
        ,(0,0,5)]

fpoints = pts1
spoints = pts2
# #stay away from the walls
# for point in pts1:
#     npoint = (point[0]+2.5,point[1]+2.5,point[2])
#     fpoints.append(npoint)

# for point in pts2:
#     npoint = (point[0]+2.5,point[1]+2.5,point[2])
#     spoints.append(npoint)

first_mission_items = []
f_mis_complete = False
second_mission_items = []
position = None
flight_mode = ""
async def main():
    global sroi
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.OUT)
    GPIO.output(PIN, False)
    drone = System()
    print("Connecting to the drone")
    await drone.connect(system_address="serial:///dev/ttyAMA0:921600")#serial:///dev/ttyAMA0:921600,udp://15540

    print("Waiting for connection")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    GPIO.output(PIN, True)
    
    print("Waiting for global position estimate")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("GPS working")
            break

    async for pos in drone.telemetry.position():
        global position
        position = pos
        print(pos)
        break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))
    
    f_mode_task = asyncio.ensure_future(
        flight_mode_handler(drone))

    running_tasks = [print_mission_progress_task,f_mode_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))
    
    # async for param in getParam(drone):
    #     if (param >= 2.5):# or True:# for sim
    #         print("Phase 1")
    #         break
    #If we want to have the coordinates from the start cmd
    # async for pos in drone.telemetry.position():
    #     global position
    #     position = pos
    #     print(pos)
    #     break
    j = 0
    for i,point in enumerate(fpoints):
        add_WP(first_mission_items,point[0]*m.cos(angle)-point[1]*m.sin(angle),point[1]*m.cos(angle)+point[0]*m.sin(angle),point[2],j)
        j += 1

    add_Wait(first_mission_items,spoints[0][0]*m.cos(angle)-spoints[0][1]*m.sin(angle),spoints[0][1]*m.cos(angle)+spoints[0][0]*m.sin(angle),spoints[0][2],j)
    j += 1
    add_ROI(first_mission_items,sroi[0]*m.cos(angle)-sroi[1]*m.sin(angle),sroi[1]*m.cos(angle)+sroi[0]*m.sin(angle),sroi[2],j)
    #add_LimSpeed(second_mission_items,0.5,1)
    j += 1
    for point in spoints:
            add_WP(first_mission_items,point[0]*m.cos(angle)-point[1]*m.sin(angle),point[1]*m.cos(angle)+point[0]*m.sin(angle),point[2],j)
            j += 1
            k = ((j//2)%3)-1
            sroi = rois[k]  
            add_ROI(first_mission_items,sroi[0]*m.cos(angle)-sroi[1]*m.sin(angle),sroi[1]*m.cos(angle)+sroi[0]*m.sin(angle),sroi[2],j)
            j += 1

    add_LAND(first_mission_items,spoints[-1][0]*m.cos(angle)-spoints[-1][1]*m.sin(angle),spoints[-1][1]*m.cos(angle)+spoints[-1][0]*m.sin(angle),spoints[-1][2],j)
    
    # async for arm_state in drone.telemetry.armed():
    #     if arm_state:
    #         print("Phase 1")
    #         #await drone.action.arm()
    #         break
    
    await drone.mission_raw.upload_mission(first_mission_items)
    print("Mission Uploaded!")
    await termination_task

    GPIO.cleanup()

async def getParam(drone):
    while True:
        await asyncio.sleep(0.1)
        yield await drone.param.get_param_float("MIS_TAKEOFF_ALT")


async def getPass():
    while True:
        await asyncio.sleep(0.1)
        yield f_mis_complete

async def getFlightMode():
    while True:
        await asyncio.sleep(0.1)
        yield flight_mode

async def print_mission_progress(drone):
    async for mission_progress in drone.mission_raw.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            global f_mis_complete
            f_mis_complete = True

def meter_to_degree(m):
    return m/111_320

def correct_longitude(long,lat):
    return long/m.cos(lat*(m.pi/180))

def add_WP(l,x,y,z,seq):
    l.append(mr.MissionItem(seq,6,16,0 if seq else 1,1,#mav_cmd_nav_waypoint
                            0.3,1.2,0,float("nan"),
                            int((position.latitude_deg + meter_to_degree(y))*1e7),
                            int((position.longitude_deg + correct_longitude(meter_to_degree(x),position.latitude_deg))*1e7),
                            z,0))

def add_Wait(l,x,y,z,seq):
    l.append(mr.MissionItem(seq,6,19,0 if seq else 1,1,#mav_cmd_nav_waypoint
                            4.0,0,float("nan"),float("nan"),
                            int((position.latitude_deg + meter_to_degree(y))*1e7),
                            int((position.longitude_deg + correct_longitude(meter_to_degree(x),position.latitude_deg))*1e7),
                            z,0))

def add_ROI(l,x,y,z,seq):
    l.append(mr.MissionItem(seq,6,195,0 if seq else 1,1,
                            0,float("nan"),float("nan"),float("nan"),
                            int((position.latitude_deg + meter_to_degree(y))*1e7),
                            int((position.longitude_deg + correct_longitude(meter_to_degree(x),position.latitude_deg))*1e7),
                            z,0))

def add_LAND(l,x,y,z,seq):
    l.append(mr.MissionItem(seq,6,21,0 if seq else 1,1,
                            0,0,float("nan"),float("nan"),
                            int((position.latitude_deg + meter_to_degree(y))*1e7),
                            int((position.longitude_deg + correct_longitude(meter_to_degree(x),position.latitude_deg))*1e7),
                            z,0))

def add_LimSpeed(l,speed,seq):
    l.append(mr.MissionItem(seq,2,178,0 if seq else 1,1,
                            0,speed,-1,0,
                            0,0,0,0))

async def flight_mode_handler(drone):
    async for f_mode in drone.telemetry.flight_mode():
        global flight_mode
        flight_mode = f_mode

async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())