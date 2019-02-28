#include "lab56danpkg/lab56.h"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
 
extern ImageConverter* ic_ptr; //global pointer from the lab56.cpp
 
#define SPIN_RATE 20  /* Hz */
 
double th1 = 0;
double th2 = 0;
double th3 = 0;
double th4 = 0;
double th5 = 0;
double th6 = 0;
 
int DigIn0 = 0;
double Ain0 = 0;
 
double home[]={120*PI/180,-90*PI/180,90*PI/180,-90*PI/180,-90*PI/180,0*PI/180};
 
std::vector<double> cur_Position (home,home+sizeof(home) / sizeof(home[0]));
 
bool ON = true;
bool OFF = false;
bool cur_io_0 = false;
 
bool current_position_set = false;
 
bool leftclickdone = 1;
bool rightclickdone = 1;
 
int gripper(ros::Publisher pub_cmd,ros::Rate rate,bool io_0)
{    
    int error = 0;
    int spincount = 0;
    int atgoal = 0;
 
    cur_io_0 = io_0;
 
    dan_ur3_driver::dancommand driver_msg;
    driver_msg.destination=cur_Position;  // Set desired position to move home
    driver_msg.a = 1.0;
    driver_msg.v = 1.0;    
    driver_msg.io_0 = io_0;
    pub_cmd.publish(driver_msg);  // publish command, but note that is possible that   
                  // the subscriber will not receive this message.
 
    while (atgoal == 0) {
        if (    fabs(th1-driver_msg.destination[0]) < 0.0005 &&
            fabs(th2-driver_msg.destination[1]) < 0.0005 &&
            fabs(th3-driver_msg.destination[2]) < 0.0005 &&
            fabs(th4-driver_msg.destination[3]) < 0.0005 &&
            fabs(th5-driver_msg.destination[4]) < 0.0005 &&
            fabs(th6-driver_msg.destination[5]) < 0.0005)
        {
            atgoal = 1;
        }
        ros::spinOnce();  // Allow other ROS functionallity to run
        rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
        if (spincount > SPIN_RATE*5) {  // if isReady does not get set within 5 second re-publish
            pub_cmd.publish(driver_msg);
            ROS_INFO("Just Published again driver_msg");
            spincount = 0;
        }
        spincount++;  // keep track of loop count
       
    }
 
    return error;
}
 
 
 
int move_arm(ros::Publisher pub_cmd , ros::Rate rate, std::vector<double> dest,double vel, double accel)
{    
    int error = 0;
    int spincount = 0;
    int atgoal = 0;
    dan_ur3_driver::dancommand driver_msg;
    driver_msg.destination=dest;  // Set desired position to move home
    driver_msg.a = accel;
    driver_msg.v = vel;    
    driver_msg.io_0 = cur_io_0;
   
    pub_cmd.publish(driver_msg);  // publish command, but note that is possible that
                      // the subscriber will not receive this message.
   
 
    while (atgoal == 0) {
        if (    fabs(th1-driver_msg.destination[0]) < 0.0005 &&
            fabs(th2-driver_msg.destination[1]) < 0.0005 &&
            fabs(th3-driver_msg.destination[2]) < 0.0005 &&
            fabs(th4-driver_msg.destination[3]) < 0.0005 &&
            fabs(th5-driver_msg.destination[4]) < 0.0005 &&
            fabs(th6-driver_msg.destination[5]) < 0.0005)
        {
            atgoal = 1;
        }
        ros::spinOnce();  // Allow other ROS functionallity to run
        rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
        if (spincount > SPIN_RATE*5) {  // if isReady does not get set within 5 second re-publish
            pub_cmd.publish(driver_msg);
            ROS_INFO("Just Published again driver_msg");
            spincount = 0;
        }
        spincount++;  // keep track of loop count
       
    }
 
    ros::spinOnce();
    return error;
}
 
/*****************************************************
* Functions in class:
* **************************************************/  
 
//constructor(don't modify)
ImageConverter::ImageConverter():it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera_node/image_raw", 1,
        &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);  
    pub_command=nh_.advertise<dan_ur3_driver::dancommand>("ur3/dancommand",10);
    sub_position=nh_.subscribe("ur3/danposition",1,&ImageConverter::position_callback,this);
    sub_input=nh_.subscribe("ur3/daninput",1,&ImageConverter::input_callback,this);
 
    ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
 
 
    while(!ros::ok()){};
 
    while (current_position_set==false) {
        ros::spinOnce();  // Allow other ROS functionallity to run
    }
 
 
    move_arm(pub_command,loop_rate, lab_invk(0.1,-.3,0.2,-90),4.0,4.0);
 
}
 
//destructor(don't modify)
ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}
 
 
void ImageConverter::position_callback(const dan_ur3_driver::danposition::ConstPtr& msg)
{
 
    th1=msg->position[0];
    th2=msg->position[1];
    th3=msg->position[2];
    th4=msg->position[3];
    th5=msg->position[4];
    th6=msg->position[5];
 
    cur_Position[0] = th1;
    cur_Position[1] = th2;
    cur_Position[2] = th3;
    cur_Position[3] = th4;
    cur_Position[4] = th5;
    cur_Position[5] = th6;
 
    current_position_set = true;
 
 
               
}
 
void ImageConverter::input_callback(const dan_ur3_driver::daninput::ConstPtr& msg)
{
 
    DigIn0 = msg->DIGIN & 0x1;  // Only look at least significant bit
    Ain0 = msg->AIN0;
    // msg->AIN1 not connected;
}
 
 
//subscriber callback function, will be called when there is a new image read by camera
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{  
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // create an gray scale version of image
    Mat gray_image;
    cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );  
    // convert to black and white img, then associate objects:  
 
    Mat bw_image;
    //Mat interm;
    adaptiveThreshold(gray_image,bw_image,255,0,0,51,15);
    //adaptiveThreshold(interm,bw_image,255,0,0,501,15);
    //adaptiveThreshold(scr,dst,MAXVALUE,adaptiveMethod,thresholdType,blocksize,C);
    //adaptiveMethod = 0, ADAPTIVE_THRESH_MEAN_C
    //thresholdType = 0, BINARY
    //blocksize
    //C constant subtracted from tz.  
   
    Mat associate_image = associateObjects(bw_image); // find associated objects
 
    // Update GUI Window
    imshow("Image window", cv_ptr->image);
    imshow("gray_scale", gray_image);
    imshow("black and white", bw_image);
    imshow("associate objects", associate_image);
    waitKey(3);
    // Output some video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}
 
/*****************************************************
     * Function for Lab 5
* **************************************************/  
// Take a grayscale image as input and return an thresholded image.
// You will implement your algorithm for calculating threshold here.
Mat ImageConverter::thresholdImage(Mat gray_img)
{
        int   totalpixels;
        Mat bw_img  = gray_img.clone(); // copy input image to a new image
        totalpixels   = gray_img.rows*gray_img.cols;// total number of pixels in image
        uchar graylevel; // use this variable to read the value of a pixel
        int zt=0; // threshold grayscale value
       
        zt = 100;  // you will be finding this automatically
 
        // threshold the image
        for(int i=0; i<totalpixels; i++)
        {
            graylevel = gray_img.data[i];  
            if(graylevel>zt) bw_img.data[i]= 255; // set rgb to 255 (white)
            else             bw_img.data[i]= 0; // set rgb to 0   (black)
        }  
    return bw_img; 
}
 
// simple function used to prevent out of bound indexing
int neighbor(void* pixellabel, int width, int height, int x, int y){
    if (x<0 || y<0 || x>=height || y>=width) return -1;
 
    return ((int (*)[width])pixellabel)[x][y];
}
 
// minimum area threshold
#define AREA_MIN 200
// maximum area threshold
#define AREA_MAX 1500
// the length of vertical/horizontal cross hatch
#define CROSS_SIZE 20
 
/*****************************************************
     * Function for Lab 5
* **************************************************/
// Take an black and white image and find the object it it, returns an associated image with different color for each image
// You will implement your algorithm for rastering here
Mat ImageConverter::associateObjects(Mat bw_img)
{
 
    //initiallize the variables you will use
    int height,width; // number of rows and colums of image
    int red, green, blue; //used to assign color of each objects
    uchar pixel; //used to read pixel value of input image
    height = bw_img.rows;
    width = bw_img.cols;
    int num=1;
    // initialize an array of labels, assigning a label number to each pixel in the image
    // this create a 2 dimensional array pixellabel[row][col]
    int pixellabel[height][width];
    for (int i=0;i<height;i++) {
        for (int j=0; j<width; j++){
            pixellabel[i][j] = 0;
        }
    }
 
    // creating a demo image of colored lines
    std::unordered_map<int, int> valid;
    std::queue<std::pair<int,int> > q;
    std::vector<std::pair<int,int> > centers;
    int tempArea;
    int majorColor = 0;
    // notice that color index starts from 1, 0 is white
    int realColor = 1;
    int tempX, tempY;
 
    // during iteration we also calculate the moments of block of pixels
    int mX, mY;
    // integration
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
          if (!bw_img.data[row * width + col]){
            if (!pixellabel[row][col]){
                // flood fill the surrounding
                q.push(std::pair<int,int>(row, col));
                // increment majorColor for block index
                majorColor++;
                tempArea = 0;
                // for calculating the center position
                mX = 0;mY = 0;
                pixellabel[row][col]=majorColor;
                while (!q.empty()){
                    // pop
                    tempX = q.front().first;
                    tempY = q.front().second;
                    mX += tempX; mY += tempY;
                    q.pop();
                    tempArea++;
                    // push surround valid pixel
                    if (!neighbor((void*)pixellabel,width,height,tempX-1,tempY) && !bw_img.data[(tempX-1) * width + tempY]){
                        q.push(std::pair<int,int>(tempX-1, tempY));
                        pixellabel[tempX-1][tempY] = majorColor;
                    }
                    if (!neighbor((void*)pixellabel,width,height,tempX,tempY-1) && !bw_img.data[tempX * width + tempY-1]){
                        q.push(std::pair<int,int>(tempX, tempY-1));
                        pixellabel[tempX][tempY-1] = majorColor;
                    }
                    if (!neighbor((void*)pixellabel,width,height,tempX+1,tempY) && !bw_img.data[(tempX+1) * width + tempY]){
                        q.push(std::pair<int,int>(tempX+1, tempY));
                        pixellabel[tempX+1][tempY] = majorColor;
                    }
                    if (!neighbor((void*)pixellabel,width,height,tempX,tempY+1) && !bw_img.data[tempX * width + tempY+1]){
                        q.push(std::pair<int,int>(tempX, tempY+1));
                        pixellabel[tempX][tempY+1] = majorColor;
                    }
                }
                // if the area is smaller than the threshold, flood fill again to erase these pixels
                if (tempArea > AREA_MIN && tempArea < AREA_MAX){
                    valid.insert(std::pair<int, int>(majorColor,realColor));
                    // choose next real color drawn, wrap around 9
                    realColor++;
                    if (realColor >= 10) realColor = 1;
                    // push center position
                    centers.push_back(std::pair<int,int>(mX/tempArea, mY/tempArea));
                }
            }
          }
        }
    }
 
    // assign UNIQUE color to each object
    Mat associate_img = Mat::zeros( bw_img.size(), CV_8UC3 ); // function will return this image
    Vec3b color;
    for(int row=0; row<height; row++)
    {
        for(int col=0; col<width; col++)
        {
        std::unordered_map<int,int>::const_iterator result = valid.find(pixellabel[row][col]);
        if(result != valid.end()){
            switch (result->second)
            {
               
                case 0:
                    red    = 255; // you can change color of each objects here
                    green = 255;
                    blue   = 255;
                    break;
                case 1:
                    red    = 255; // you can change color of each objects here
                    green  = 0;
                    blue   = 0;
                    break;
                case 2:
                    red    = 0;
                    green  = 255;
                    blue   = 0;
                    break;
                case 3:
                    red    = 0;
                    green  = 0;
                    blue   = 255;
                    break;
                case 4:
                    red    = 255;
                    green  = 255;
                    blue   = 0;
                    break;
                case 5:
                    red    = 255;
                    green  = 0;
                    blue   = 255;
                    break;
                case 6:
                    red    = 0;
                    green  = 255;
                    blue   = 255;
                    break;
                case 7:
                    red    = 128;
                    green  = 128;
                    blue   = 0;
                    break;
                case 8:
                    red    = 128;
                    green  = 0;
                    blue   = 128;
                    break;
                case 9:
                    red    = 0;
                    green  = 128;
                    blue   = 128;
                    break;
                default:
                    red    = 0;
                    green = 0;
                    blue   = 0;
                    break;                 
            }
            }else{
                red = 255;
                green = 255;
                blue = 255;
            }
 
            color[0] = blue;
            color[1] = green;
            color[2] = red;
            associate_img.at<Vec3b>(Point(col,row)) = color;
        }
    }
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    for (std::pair<int, int> center : centers){
        // get the center position from queue
        tempX = center.first;
        tempY = center.second;
        associate_img.at<Vec3b>(Point(tempY, tempX)) = color;
 
        // draw the crosshair, notice that boundary rejection
        for (int i = -(CROSS_SIZE/2); i < (CROSS_SIZE/2); ++i){
            // vertical
            if ( (tempX+i)>0 || (tempX+i)<height)
                associate_img.at<Vec3b>(Point(tempY, tempX+i)) = color;
            // horizontal
            if ( (tempY+i)>0 || (tempY+i)<width)
                associate_img.at<Vec3b>(Point(tempY+i, tempX)) = color;
        }
    }
   
    return associate_img;
}
 
/*****************************************************
    *Function for Lab 6
 * **************************************************/
 //This is a call back function of mouse click, it will be called when there's a click on the video window.
 //You will write your coordinate transformation in onClick function.
 //By calling onClick, you can use the variables calculated in the class function directly and use publisher
 //initialized in constructor to control the robot.
 //lab4 and lab3 functions can be used since it is included in the "lab4.h"
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    ic_ptr->onClick(event,x,y,flags,userdata);
}
 
// center pixel of camera
#define COR (240)
#define COC (320)
 
// one block distance in camera pixels
#define PIXEL_D (344-270)
// one block in world centimeter
#define WORLD_D (10)
 
// calculated scale
#define SCALE (((double)PIXEL_D)/WORLD_D)
 
// world frame origin
#define T_X (310)
#define T_Y (460)
// tilted angle of camera
#define TILT (-0.015)
 
void ImageConverter::onClick(int event,int x, int y, int flags, void* userdata)
{
    // calculate transformation first
    double t_x = (T_X - COR)/SCALE;
    double t_y = (T_Y - COC)/SCALE;
 
    // first pixel position to camera frame coordinates
    double x_c = (x - COR)/SCALE;
    double y_c = (y - COC)/SCALE;
 
    // change camera coordinates to inverted world coordinates
    // note that we invert those in the movement command
    double x_w = (x_c - t_x) * cos(TILT) + (y_c - t_y) * sin(TILT);
    double y_w = (x_c - t_x) * (-sin(TILT)) + (y_c - t_y) * cos(TILT);
 
    //std::cout<< "click point in world coordinates, X:" << x_w << " Y:" << y_w << std::endl;
 
    // If the robot is holding a block, place it at the designated row and column.
    if  ( event == EVENT_LBUTTONDOWN ) //if left click, do nothing other than printing the clicked point
    {  
        if (leftclickdone == 1) {
            leftclickdone = 0;  // code started
            ROS_INFO_STREAM("left click:  (" << x << ", " << y << ")");  //the point you clicked
           
            ros::Rate loop_rate(SPIN_RATE);
 
            // move to the x-y position but a little bit higher in z
            move_arm(pub_command,loop_rate, lab_invk(-y_w/100.0,-x_w/100.0,0.20,-90),4.0,4.0);
            // go down
            move_arm(pub_command,loop_rate, lab_invk(-y_w/100.0,-x_w/100.0,0.02,-90),4.0,4.0);
            // get the block
            gripper(pub_command,loop_rate,ON);
            // move up
            move_arm(pub_command,loop_rate, lab_invk(-y_w/100.0,-x_w/100.0,0.20,-90),4.0,4.0);
            // go to initial position
            move_arm(pub_command,loop_rate, lab_invk(0.1,-.3,0.2,-90),4.0,4.0);
            leftclickdone = 1; // code finished
            ros::spin();    // spin so that the video stream doesn't freeze
           
 
        } else {
            ROS_INFO_STREAM("Previous Left Click not finshed, IGNORING this Click");
        }
    } else if  ( event == EVENT_RBUTTONDOWN )//if right click, find nearest centroid,
    {
        if (rightclickdone == 1) {  // if previous right click not finished ignore
            rightclickdone = 0;  // starting code
            ROS_INFO_STREAM("right click:  (" << x << ", " << y << ")");  //the point you clicked
 
            // put your right click code here
            ros::Rate loop_rate(SPIN_RATE);
 
            // move to the x-y position but a little bit higher in z
            move_arm(pub_command,loop_rate, lab_invk(-y_w/100.0,-x_w/100.0,0.20,-90),4.0,4.0);
            // go down
            move_arm(pub_command,loop_rate, lab_invk(-y_w/100.0,-x_w/100.0,0.02,-90),4.0,4.0);
            // release gripper
            gripper(pub_command,loop_rate,OFF);
            // move up
            move_arm(pub_command,loop_rate, lab_invk(-y_w/100.0,-x_w/100.0,0.20,-90),4.0,4.0);
            // back to initial position
            move_arm(pub_command,loop_rate, lab_invk(0.1,-.3,0.2,-90),4.0,4.0);
           
            rightclickdone = 1; // code finished   
            ros::spin();    // spin so that the video stream doesn't freeze
           
        } else {
            ROS_INFO_STREAM("Previous Right Click not finshed, IGNORING this Click");
        }
    }
}
