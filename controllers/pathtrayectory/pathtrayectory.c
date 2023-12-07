#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXIMUM_NUMBER_OF_COORDINATES 50  // Size of the history.
#define REFRESH_FACTOR 2                 // Refresh the trail every REFRESH_FACTOR * WorldInfo.basicTimeStep.

// Create the trail shape with the correct number of coordinates.
static void create_trail_shape() {
  // If TRAIL exists in the world then silently remove it.
  WbNodeRef existing_trail = wb_supervisor_node_get_from_def("TRAIL");
  if (existing_trail)
    wb_supervisor_node_remove(existing_trail);

  int i;
  char trail_string[0x10000] = "\0";  // Initialize a big string which will contain the TRAIL node.

  // Create the TRAIL Shape.
  strcat(trail_string, "DEF TRAIL Shape {\n");
  strcat(trail_string, "  appearance Appearance {\n");
  strcat(trail_string, "    material Material {\n");
  strcat(trail_string, "      diffuseColor 0 1 0\n");
  strcat(trail_string, "      emissiveColor 0 0.5 0\n");
  strcat(trail_string, "    }\n");
  strcat(trail_string, "  }\n");
  strcat(trail_string, "  geometry DEF TRAIL_LINE_SET IndexedLineSet {\n");
  strcat(trail_string, "    coord Coordinate {\n");
  strcat(trail_string, "      point [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trail_string, "      0 0 0\n");
  strcat(trail_string, "      ]\n");
  strcat(trail_string, "    }\n");
  strcat(trail_string, "    coordIndex [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(trail_string, "      0 0 -1\n");
  strcat(trail_string, "    ]\n");
  strcat(trail_string, "  }\n");
  strcat(trail_string, "}\n");

  // Import TRAIL and append it as the world root nodes.
  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, trail_string);
}

int main(int argc, char **argv) {
  wb_robot_init();

  // Set the refresh rate of this controller, and so, set the refresh rate of the line set.
  int time_step = (int)wb_robot_get_basic_time_step();  // i.e. `WorldInfo.basicTimeStep`
  time_step *= REFRESH_FACTOR;

  // Get the target object node, i.e. the TARGET Transform in the E-puck turretSlot field.
  WbNodeRef target_node = wb_supervisor_node_get_from_def("TARGET");

  // Create the TRAIL Shape which will contain the green line set.
  create_trail_shape();

  // Get interesting references to the TRAIL subnodes.
  WbNodeRef trail_line_set_node = wb_supervisor_node_get_from_def("TRAIL_LINE_SET");
  WbNodeRef coordinates_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(trail_line_set_node, "coord"));
  WbFieldRef point_field = wb_supervisor_node_get_field(coordinates_node, "point");
  WbFieldRef coord_index_field = wb_supervisor_node_get_field(trail_line_set_node, "coordIndex");

  int index = 0;           // This points to the current position to be drawn.
  bool first_step = true;  // Only equals to true during the first step.

  // Main loop.
  while (wb_robot_step(time_step) != -1) {
    // Get the current target translation.
    const double *target_translation = wb_supervisor_node_get_position(target_node);

    // Only add the new target translation to the line set if it's not [0, 0, 0].
    if (target_translation[0] != 0.0 || target_translation[1] != 0.0 || target_translation[2] != 0.0) {
      // Add the new target translation to the line set.
      wb_supervisor_field_set_mf_vec3f(point_field, index, target_translation);

      // Update the line set indices.
      if (index > 0) {
        // Link successive indices.
        wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (index - 1), index - 1);
        wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (index - 1) + 1, index);
      } else if (index == 0 && first_step == false) {
        // Link the first and the last indices.
        wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1), 0);
        wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1) + 1,
                                         MAXIMUM_NUMBER_OF_COORDINATES - 1);
      }
      // Unset the next indices.
      wb_supervisor_field_set_mf_int32(coord_index_field, 3 * index, index);
      wb_supervisor_field_set_mf_int32(coord_index_field, 3 * index + 1, index);
    } else {
      // If the target translation is [0, 0, 0], skip adding it to the line set and continue to the next iteration.
      continue;
    }
    
    //printf("Hola, este es un mensaje en C.\n");
    // Update global variables.
    first_step = false;
    index++;
    index = index % MAXIMUM_NUMBER_OF_COORDINATES;
  }

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
