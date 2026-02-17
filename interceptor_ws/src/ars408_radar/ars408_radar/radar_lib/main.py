import time
from radar import Radar 
from filters_cfgs import filters_obj_0x202, cfg_radar_0x200  
from plot_radar import init_topdown_plot, plot_objects_topdown
#simple test program 
def main():

    fig, ax = init_topdown_plot()
    radar = Radar(print_radar_state=1)  

    # Send configs
    radar.send_cfg_0x200(cfg_radar_0x200)
    radar.send_objects_filter(filters_obj_0x202)

    # Read filters configs
    # radar.read_objects_filter()
    time.sleep(0.1)

    # main loop
    try:
        for event in radar.run():
            if event["type"] == "0x201":
                print("Radar status:", event["data"])
            if event["type"] == "0x60":
                objects = event["data"]["objects"]
                print("Objects detected:", objects)
                plot_objects_topdown(objects, ax)

    except KeyboardInterrupt:
        print("Arrêt radar")
    finally:
        radar.shutdown()


if __name__ == "__main__":
    main()
