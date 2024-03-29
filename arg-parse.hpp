#ifndef ARG_PARSE_HPP
#define ARG_PARSE_HPP

#include <argp.h>
#include <string>


enum opt {
    OPT_FOURCC = 1000,
    OPT_SAVE_IMG_DIR,
    OPT_SAVE_IMG_PER,
    OPT_COMPENZATION_IMG,
};

/* Command line options */
struct cmd_arguments{
    bool enter_poi = false;
    std::string poi_export_path;
    std::string poi_import_path;
    std::string show_poi_path;
    std::string license_file;
    std::string vid_in_path;
    std::string vid_out_path;
    std::string fourcc = "HFYU";
    int display_delay_us = 0;
    std::string poi_csv_file;
    bool save_img = false;
    std::string save_img_dir;
    double save_img_period = 0;
    bool webserver_active = false;
    enum class tracking {on, off, once, background};
    tracking tracking = tracking::off;
    std::string heat_sources_border_points;
    std::string compenzation_img;
};

extern struct argp argp;

#endif // ARG_PARSE_HPP
