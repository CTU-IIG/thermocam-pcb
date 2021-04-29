#include "arg-parse.hpp"
#include "version.h"
#include <string.h>

using namespace std;

static error_t parse_opt(int key, char *arg, struct argp_state *argp_state)
{
    cmd_arguments &args = *reinterpret_cast<cmd_arguments*>(argp_state->input);

    switch (key) {
    case 'e':
        args.enter_poi = true;
        if(arg != NULL)
            args.poi_export_path = arg;
        break;
    case 'p':
        args.poi_import_path = arg;
        break;
    case 's':
        args.show_poi_path = arg;
        break;
    case 'l':
        args.license_dir = arg;
        break;
    case 'r':
        args.vid_out_path = arg;
        break;
    case OPT_FOURCC:
        if (strlen(arg) != 4) {
            argp_error(argp_state, "fourcc code must have 4 characters");
            return EINVAL;
        }
        args.fourcc = arg;
        break;
    case 'v':
        args.vid_in_path = arg;
        break;
    case 'c':
        args.poi_csv_file = arg;
        break;
    case 'd':
        args.display_delay_us = atof(arg) * 1000000;
        break;
    case 't':
        if (!arg) {
            args.track = cmd_arguments::tracking::on;
        } else if (string(arg) == "once") {
            args.track = cmd_arguments::tracking::once;
        } else if (string(arg) == "bg" || string(arg) == "background") {
            args.track = cmd_arguments::tracking::background_first_iter;
        } else {
            argp_error(argp_state, "Unknown tracking mode: %s", arg);
            return EINVAL;
        }
        break;
    case 'h':
        if (args.track == cmd_arguments::tracking::off)
            args.track = cmd_arguments::tracking::on;
        args.heat_sources_border_points = arg;
        break;
    case OPT_SAVE_IMG_DIR:
        args.save_img_dir = arg;
        args.save_img = true;
        break;
    case OPT_SAVE_IMG_PER:
        args.save_img_period = atof(arg);
        args.save_img = true;
        break;
    case 'w':
        args.webserver_active = true;
        break;
    case ARGP_KEY_END:
        if (args.license_dir.empty())
            args.license_dir = ".";
        if (args.save_img && args.save_img_dir.empty())
            args.save_img_dir = ".";
        if (args.save_img &&args.save_img_period == 0)
            args.save_img_period = 1;
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

/* The options we understand. */
static struct argp_option options[] = {
    { "enter-poi",       'e', "FILE",        OPTION_ARG_OPTIONAL, "Enter Points of interest by hand, optionally save them to json file at supplied path." },
    { "poi-path",        'p', "FILE",        0, "Path to config file containing saved POIs." },
    { "show-poi",        's', "FILE",        0, "Show camera image taken at saving POIs." },
    { "license-dir",     'l', "FILE",        0, "Path to directory containing WIC license file.\n\".\" by default." },
    { "record-video",    'r', "FILE",        0, "Record video and store it with entered filename"},
    { "fourcc",          OPT_FOURCC, "CODE", 0, "4-letter code for video codec used by -r (e.g. MJPG, h264), default: HFYU"},
    { "load-video",      'v', "FILE",        0, "Load and process video instead of camera feed"},
    { "csv-log",         'c', "FILE",        0, "Log temperature of POIs to a csv file instead of printing them to stdout."},
    { "save-img-dir",    OPT_SAVE_IMG_DIR, "DIR",  0, "Target directory for saving an image with POIs every \"save-img-period\" seconds.\n\".\" by default."},
    { "save-img-period", OPT_SAVE_IMG_PER, "SECS", 0, "Period for saving an image with POIs to \"save-img-dir\".\n1s by default."},
    { "track-points",    't', "once",        OPTION_ARG_OPTIONAL, "Turn on tracking of points. If \"once\" is specified, tacking happens only for the first image. "
                                                                  "This allows faster processing if the board doesn't move. If \"bg\" is specified, calculations run in separate thread."},
    { "heat-sources",    'h', "PT_LIST",     0, "Enables heat sources detection. PT_LIST is a comma separated list of names of 4 points (specified with -p) that define detection area. Implies -t."},
    { "delay",           'd', "NUM",         0, "Set delay between each measurement/display in seconds."},
    { "webserver",       'w', 0,             0, "Start webserver to display image and temperatures."},
    { 0 }
};

const char * argp_program_bug_address = "https://github.com/CTU-IIG/thermocam-pcb/issues";
const char * argp_program_version = "thermocam-pcb " GIT_VERSION;

/* Our argp parser. */
struct argp argp = {
    options, parse_opt, "[--] COMMAND...",

    "Displays thermocamera image and entered points of interest and their "
    "temperature. Writes the temperatures of entered POIs to stdout."

    "\v"

    "Requires path to directory containing WIC license file to run with camera.\n\n"

    "Controls:\n"
    "Tab                - Change view  (Full | Temperature only | Legend)\n"
    "Mouse click (left) - Enter point  (only with --enter-poi)\n"
    "Backspace          - Remove point (only with --enter-poi)\n"
    "Esc                - Exit program\n"
};
