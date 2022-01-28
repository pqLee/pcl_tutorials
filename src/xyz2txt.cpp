#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
 
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
 
void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.xyz output.pcd\n", argv[0]);
}
 
bool
loadCloud (const string &filename, PointCloud<PointXYZRGB> &cloud)
{
  ifstream fs;
  fs.open (filename.c_str (), ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
    fs.close ();
    return (false);
  }
 
  string line;
  vector<string> st;
 
  while (!fs.eof ())
  {
    getline (fs, line);
    // Ignore empty lines
    if (line.empty())
      continue;
 
    // Tokenize the line
    boost::trim (line);
    boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
 
    if (st.size () != 7)
      continue;
 
    pcl::PointXYZRGB point;
    point.x = float (atof (st[0].c_str ())); 
    point.y = float (atof (st[1].c_str ())); 
    point.z = float (atof (st[2].c_str ()));
    point.r = uint8_t (atof (st[4].c_str ()));
    point.g = uint8_t (atof (st[5].c_str ()));
    point.b = uint8_t (atof (st[6].c_str ()));
    cloud.push_back (point);
 
  }
  fs.close ();
 
  cloud.width = uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
  return (true);
}
 
int
main (int argc, char** argv)
{
  print_info ("Convert a simple XYZ file to PCD format. For more information, use: %s -h\n", argv[0]);
 
  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }
 
  // Parse the command line arguments for .pcd and .ply files
  vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  vector<int> xyz_file_indices = parse_file_extension_argument (argc, argv, ".xyz");
  if (pcd_file_indices.size () != 1 || xyz_file_indices.size () != 1)
  {
    print_error ("Need one input XYZ file and one output PCD file.\n");
    return (-1);
  }
 
  // Load the first file
  PointCloud<PointXYZRGB> cloud;
  if (!loadCloud (argv[xyz_file_indices[0]], cloud)) 
    return (-1);
 
  // Convert to PCD and save
  pcl::io::savePCDFileASCII (argv[pcd_file_indices[0]], cloud);
}
