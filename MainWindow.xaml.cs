using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Emgu.CV;
using Emgu.CV.Structure;
using Microsoft.Kinect;
using Mat = Emgu.CV.Image<Emgu.CV.Structure.Bgr, byte>;

namespace KinectGettingStarted
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    class API
    {
        [DllImport("GDI32.dll")]
        public static extern int DeleteObject(IntPtr hObject);
    }
    public partial class MainWindow : Window
    {
        KinectSensor _kinectNui;
        int totalFrames = 0;
        int lastFrames = 0;
        DateTime lastTime = DateTime.MaxValue;
        short[] shortpixeldata; //newcode
        byte[] pixeldata;
        Image<Gray, Byte> Dest;
        Bitmap bmap, bmp;
        Skeleton[] skeletonData;
        bool edgesAv = false;
        bool sideways = false;
        float[] chest = new float[10];
        float[] waist = new float[10];
        float[] hip = new float[10];
        float chestSize, waistSize, hipSize;

        //The bitmap that will contain the actual converted depth into an image
        WriteableBitmap outputBitmap;
        private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        byte[] depthFrame32;
        //Format of the last Depth Image.
        //This changes when you first run the program or whenever you minimize the window
        private DepthImageFormat lastImageFormat;

        //Identify each color layer on the R G B
        private const int RedIndex = 2;
        private const int GreenIndex = 1;
        private const int BlueIndex = 0;
        int counter = 0;
          
        public MainWindow()
        {
            InitializeComponent();
            btnCameraDown.IsEnabled = false;
            btnCameraUp.IsEnabled = false;
        }

        private void InitializeNui()
        {
            try
            {
                //•	Declares _kinectNui as a KinectSensor object, which represents the Kinect sensor instance.
                ////_kinectNui = new KinectSensor();
                _kinectNui = KinectSensor.KinectSensors[0];
                //Open the video and depth streams, and sets up the event handlers that the runtime calls when a video, depth, or skeleton frame is ready
                //An application must initialize the Kinect sensor
                // _kinectNui.ColorStream.Enable();
                //_kinectNui.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                //_kinectNui.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(ColorImageFrameReady);

                _kinectNui.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                _kinectNui.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(DepthFrameReady);

                _kinectNui.SkeletonStream.Enable();
                skeletonData = new Skeleton[_kinectNui.SkeletonStream.FrameSkeletonArrayLength];
                _kinectNui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(SkeletonFrameReady);
                _kinectNui.Start();


                //To stream color images:
                //  •	The options must include UseColor.
                //  •	Valid image resolutions are Resolution1280x1024 and Resolution640x480.
                //  •	Valid image types are Color, ColorYUV, and ColorYUVRaw.
                //    _kinectNui.VideoStream.Open(ImageStreamType.Video, 2,ImageResolution.Resolution640x480, ImageType.ColorYuv);


                lastTime = DateTime.Now;

                // Camera _cam;
                //  _cam = _kinectNui.NuiCamera;
                txtCameraName.Text = _kinectNui.UniqueKinectId;
                //_kinectNui.VideoFrameReady += new EventHandler<ImageFrameReadyEventArgs>(NuiVideoFrameReady);
                btnStop.IsEnabled = true;
                btnStart.IsEnabled = false;
                btnCameraDown.IsEnabled = true;
                btnCameraUp.IsEnabled = true;

            }
            catch (InvalidOperationException ex)
            {
                MessageBox.Show(ex.Message);
            }
            catch (ArgumentOutOfRangeException e)
            {
                btnStop.IsEnabled = false;
                btnStart.IsEnabled = true;
                MessageBox.Show("No Kinect detected. Please connect a Kinect Sensor and retry.");

            }

           

        }

        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
           // InitializeNui();
            // DispImg();
            btnStart.IsEnabled = true;
            btnStop.IsEnabled = false;
        }

        void CalculateFps()
        {
            ++totalFrames;

            var cur = DateTime.Now;
            if (cur.Subtract(lastTime) > TimeSpan.FromSeconds(1))
            {
                int frameDiff = totalFrames - lastFrames;
                lastFrames = totalFrames;
                lastTime = cur;
                frameRate.Text = frameDiff.ToString() + " fps";
            }
            //check if connected, then get elevation angle
            if (_kinectNui.IsRunning)
            {
                txtElevationAngle.Content = _kinectNui.ElevationAngle.ToString();
            }

        }

        void SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            // Open the Skeleton frame
            using (SkeletonFrame sF = e.OpenSkeletonFrame())
            {

                // check that a frame is available
                if (sF != null && this.skeletonData != null)
                {
                    // get the skeletal information in this frame
                    sF.CopySkeletonDataTo(this.skeletonData);
                    foreach (Skeleton sk in this.skeletonData)
                    {
                        if (sk.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            TraceTrackedSkeletonJoints(sk.Joints);
                        }
                    }
                }
                /*foreach(Skeleton sk in this.skeletonData)
                {
                    if (sk.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        TraceTrackedSkeletonJoints(sk.Joints);
                    }
                }*/
            }

        }

        void TraceTrackedSkeletonJoints(JointCollection jCollection)
        {
            if(counter<10){
            //Shoulders--
            //shoulder extra dist to be added:around +0.05m on each side, so +0.10m front
            float shLx = jCollection[JointType.ShoulderLeft].Position.X;
            float shLy = jCollection[JointType.ShoulderLeft].Position.Y;
            float shRx = jCollection[JointType.ShoulderRight].Position.X;
            float shRy = jCollection[JointType.ShoulderRight].Position.Y;
            float shAdj = 0.1f;
            //Hips--
            //Hips extra addn?? estimate. around 5-6cm each side?
            float HipRx = jCollection[JointType.HipRight].Position.X;
            float HipRy = jCollection[JointType.HipRight].Position.Y;
            float HipLx = jCollection[JointType.HipLeft].Position.X;
            float HipLy = jCollection[JointType.HipLeft].Position.Y;
            float HipAdj = 0.15f;
            //Arm--
            //no extra adjustment reqd?
            float ElbowRx = jCollection[JointType.ElbowRight].Position.X;
            float ElbowRy = jCollection[JointType.ElbowRight].Position.Y;
            float ElbowLx = jCollection[JointType.ElbowLeft].Position.X;
            float ElbowLy = jCollection[JointType.ElbowLeft].Position.Y;

            // System.Diagnostics.Debug.WriteLine("Shoulder:left x{0} y{1},right x{2} y{3}", shLx,shLy,shRx,shRy);
            double ShDist = Math.Sqrt((shLx - shRx) * (shLx - shRx) + (shLy - shRy) * (shLy - shRy)); //Shoulder dist bw L & R
            double HipDist = Math.Sqrt((HipLx - HipRx) * (HipLx - HipRx) + (HipLy - HipRy) * (HipLy - HipRy)); //Waistdist
            double elbpitL = Math.Sqrt((ElbowLx - shLx) * (ElbowLx - shLx) + (ElbowLy - shLy) * (ElbowLy - shLy)); //Left elbow to armpit dist
            double elbpitR = Math.Sqrt((ElbowRx - shRx) * (ElbowRx - shRx) + (ElbowRy - shRy) * (ElbowRy - shRy)); //Right elbow to armpit dist
            double elbpit = (elbpitL + elbpitR) / 2; //avg elbow-armpit dist
            double manHeight = elbpit * 8; //RW distVitruvian:elbpit=H/8
            double head2chest = manHeight / 5; //RW dist Vitruvian: top of head to chest:H/6

            DepthImagePoint dip_ShL = _kinectNui.CoordinateMapper.MapSkeletonPointToDepthPoint(jCollection[JointType.ShoulderLeft].Position, DepthImageFormat.Resolution640x480Fps30);
            DepthImagePoint dip_ShR = _kinectNui.CoordinateMapper.MapSkeletonPointToDepthPoint(jCollection[JointType.ShoulderRight].Position, DepthImageFormat.Resolution640x480Fps30);
            DepthImagePoint dip_manHead = _kinectNui.CoordinateMapper.MapSkeletonPointToDepthPoint(jCollection[JointType.Head].Position, DepthImageFormat.Resolution640x480Fps30);

            int DnDist = dip_ShL.X - dip_ShR.X; //neck/shoulder distance in pixels
            //System.Diagnostics.Debug.WriteLine("Pixel Distance bw neck/shoulder them: px{0}", DnDist);
            //Here is where the magic begins
            float propAdj = 0.827686678f;
            float propconst = Math.Abs((float)ShDist / DnDist);  //proportionality constant to convert every other value we get from depth image
            propconst *= propAdj;
            //System.Diagnostics.Debug.WriteLine("Dndist*propconst: {0}",DnDist*propconst);

            DepthImagePoint dp = _kinectNui.CoordinateMapper.MapSkeletonPointToDepthPoint(jCollection[JointType.HipCenter].Position, DepthImageFormat.Resolution640x480Fps30);
            //DepthImagePoint dHipL, dHipR;
            //SkeletonPoint sHipL, sHipR;
            //proceed when edges image is available:
            if (edgesAv)
            {
                Mat m1 = new Mat(Dest.Bitmap);

                // System.Diagnostics.Debug.WriteLine("Rows :{0},Cols:{1}", m1.Rows, m1.Cols);
                //int cHip_xS = (int)jCollection[JointType.HipCenter].Position.X, cHip_yS = (int)jCollection[JointType.HipCenter].Position.Y;
                float cHip_x = dp.X, cHip_y = dp.Y;
                // System.Diagnostics.Debug.WriteLine("hip pixel posn x :{0},hip y:{1}", cHip_x, cHip_y);
                float leftHi = -1.0f, leftCi = -1.0f, rightHi = -1.0f, rightCi = -1.0f, headtop = -1.0f;
                for (float col = cHip_x; col > 0; col--)
                {
                    //assuming >240 means sufficiently white pixel
                    if (m1.Data[(int)cHip_y, (int)col, 0] >= 240)
                    {
                        // System.Diagnostics.Debug.WriteLine("White people");
                        leftHi = col;
                        break;//exit after first instance of white edge
                    }
                }

                for (float col = cHip_x; col <= m1.Cols; col++)
                {
                    if (m1.Data[(int)cHip_y, (int)col, 0] >= 240)
                    {
                        rightHi = col;
                        break;
                    }
                }
                //posn of top of head calc
                //asuuming row origin is at top
                for (float row = dip_manHead.Y; row > 0; row--)
                {
                    if (m1.Data[(int)row, (int)dip_manHead.X, 0] >= 240)
                    {
                        headtop = row;
                        break;
                    }
                }
            


                float chestCx = dip_manHead.X, chestCy = (float)(dip_manHead.Y + (head2chest) / propconst); //chestCentre y posn=top of head+ H/6 in px
                for (float col = chestCx; col > 0; col--)
                {
                    //assuming >240 means sufficiently white pixel
                    if (m1.Data[(int)chestCy, (int)col, 0] >= 240)
                    {
                        // System.Diagnostics.Debug.WriteLine("White people");
                        leftCi = col;
                        break;//exit after first instance of white edge
                    }
                }

                for (float col = chestCx; col < m1.Cols; col++)
                {
                    if (m1.Data[(int)chestCy, (int)col, 0] >= 240)
                    {
                        rightCi = col;
                        break;
                    }
                }

                float pxChestSize = (float)Math.Abs(rightCi - leftCi); //chest len in px
                float RWChestLen = propconst * pxChestSize; //chest len in Real world dimensions (meters)
                float pxHipSize = (float)Math.Abs(rightHi - leftHi);
                float RWHipSize = propconst * pxHipSize; //conversion of hip length in pixels to meters
                chest[counter] = RWChestLen;
                waist[counter] = (float)(HipDist + HipAdj);
                hip[counter] = RWHipSize;


            


                counter++;
            }
                if (counter == 10)
                {
                    float Chestsum= 0;
                    float Waistsum= 0;
                    float Hipsum= 0;
                    _kinectNui.SkeletonFrameReady -= SkeletonFrameReady;
                    _kinectNui.DepthFrameReady -= DepthFrameReady;
                    System.Diagnostics.Debug.WriteLine("--##################-");
                    for (int i = 0; i < 10; i++)
                    {
                        Chestsum += chest[i];
                        Waistsum += waist[i];
                        Hipsum += hip[i];
                    }
                    Chestsum = Chestsum / 10;
                    Waistsum = Waistsum / 10;
                    Hipsum = Hipsum / 10;
                    System.Diagnostics.Debug.WriteLine("Chest Avg, Waist Avg, Hip Avg is {0}, {1}, {2} respectively", Chestsum, Waistsum, Hipsum);


                    MessageBoxResult r1;
                    if(!sideways){
                       r1 = MessageBox.Show("Please turn sideways while facing the Kinect camera", "Information", MessageBoxButton.OK);
                       if (r1 == MessageBoxResult.OK)
                       {
                           chestSize = 2 * Chestsum;
                           waistSize = 2 * Waistsum;
                           hipSize = 2 * Hipsum;
                           counter = 0;
                           sideways = true;
                           _kinectNui.SkeletonFrameReady += SkeletonFrameReady;
                           _kinectNui.DepthFrameReady += DepthFrameReady;
                       }
                    }
                    else
                       {
                           chestSize += Chestsum;
                           waistSize += Waistsum;
                           hipSize += Hipsum;
                           chestSize *= 100;
                           waistSize *= 100;
                           hipSize *= 100;
                           System.Diagnostics.Debug.WriteLine("Program has worked successfully.");
                           System.Diagnostics.Debug.WriteLine("{0} {1} {2}", chestSize, waistSize, hipSize);
                           _kinectNui.Stop();
                           computeSize();
                       }
                    
                }
                // System.Diagnostics.Debug.WriteLine("Length of waist(hip) (meter):{0}", 100*(sHipL.X - sHipR.X)); //cant get distance directly by subtracting for spatial points, need to apply x2-x1^2+y2-y1^2 formula

            }
        }

        void computeSize()
        {
            if (chestSize < 91.5 && waistSize < 76 && hipSize < 91.5)
                MessageBox.Show("Your size is Small");
            else if (chestSize < 101.5 && waistSize < 86.5 && hipSize < 101.5)
                MessageBox.Show("Your size is Medium");
            else if (chestSize < 112 && waistSize < 96.5 && hipSize < 112)
                MessageBox.Show("Your size is Large");
            else if (chestSize < 122 && waistSize < 106.5 && hipSize < 119.5)
                MessageBox.Show("Your size if Extra Large");
            else if (chestSize < 132 && waistSize < 117 && hipSize < 127)
                MessageBox.Show("Your size is Extra Extra Large");
            else
                MessageBox.Show("Your size is 3X Large");
        }

        void ColorImageFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            // PlanarImage Image = e.ImageFrame.Image;
            ColorImageFrame Image;
            //BitmapImage bitmap;
            using (Image = e.OpenColorImageFrame())
            {
                

                Bitmap bm = ImageToBitmap(Image);
                if (bm != null)
                {
                    //openCV image:?
                    Image<Bgr, Byte> currentFrame = new Image<Bgr, Byte>(bm);  // return a OpenCv image
                    // Image<Bgr, Byte> currentFrame = ToOpenCVImage<Bgr, Byte>(bitmap);

                    //CV processing
                    Image<Gray, Byte> grayImage = currentFrame.Convert<Gray, Byte>().PyrDown().PyrUp();
                    Image<Gray, Byte> Dest = new Image<Gray, Byte>(grayImage.Size);
                    CvInvoke.cvCanny(grayImage, Dest, 10, 60, 3);
                    // Image<Gray, Byte> SobelHorizontal = new Image<Gray, Byte>(grayImage.Size);
                    // CvInvoke.cvSobel(Dest, SobelHorizontal, 1, 0, 3); // introduces exception

                    image.Source = ToBitmapSource(currentFrame);
                    image1.Source = ToBitmapSource(Dest);

                    CalculateFps();
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine("bitmap empty :/");
                }
            }
        }

        void DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            // PlanarImage Image = e.ImageFrame.Image;
            DepthImageFrame Image;
            Bitmap bm;
            using (Image = e.OpenDepthImageFrame())
            {
                

                if (Image != null)
                {
                    //Check if the format of the image has changed.
                    //This always happens when you run the program for the first time and every time you minimize the window
                    //bool NewFormat = this.lastImageFormat != Image.Format;
                    //Copy the stream to its short version



                    //Copy the RGB matrix to the bitmap to make it visible
                    // this.outputBitmap.WritePixels(new Int32Rect(0, 0, Image.Width, Image.Height), convertedDepthBits,  Image.Width * Bgr32BytesPerPixel,  0);
                    //Update the Format
                    // this.lastImageFormat = Image.Format;
                    //if (NewFormat)
                    //{
                    //Update the image to the new format
                    this.shortpixeldata = new short[Image.PixelDataLength];
                    this.depthFrame32 = new byte[Image.Width * Image.Height * Bgr32BytesPerPixel]; //w,H,Bgr32BytesPerPixel
                    // System.Diagnostics.Debug.WriteLine("const :{0}", Bgr32BytesPerPixel);
                    //Create the new Bitmap
                    bmp = new Bitmap(Image.Width, Image.Height, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
                    Image.CopyPixelDataTo(this.shortpixeldata);
                    //Convert the pixel data into its RGB Version.
                    //Here is where the magic happens
                    byte[] convertedDepthBits = this.ConvertDepthFrame(this.shortpixeldata, ((KinectSensor)sender).DepthStream);
                    // System.Diagnostics.Debug.WriteLine("converteddepthbits length: {0}", convertedDepthBits.Length);

                    BitmapData bmapdata = bmp.LockBits(
                                        new System.Drawing.Rectangle(0, 0, Image.Width, Image.Height),
                                        ImageLockMode.WriteOnly,
                                        bmp.PixelFormat);


                    IntPtr ptr = bmapdata.Scan0;
                    Marshal.Copy(convertedDepthBits, 0, ptr, Image.PixelDataLength * 4);

                    bmp.UnlockBits(bmapdata);


                    //  this.outputBitmap = new WriteableBitmap(  Image.Width,   Image.Height,  96, 96, PixelFormats.Bgr32,   null);

                    MemoryStream ms1 = new MemoryStream();
                    bmp.Save(ms1, System.Drawing.Imaging.ImageFormat.Jpeg);
                    System.Windows.Media.Imaging.BitmapImage bImg = new System.Windows.Media.Imaging.BitmapImage();
                    bImg.BeginInit();
                    bImg.StreamSource = new MemoryStream(ms1.ToArray());
                    bImg.EndInit();
                    image.Source = bImg;

                    if (bmp != null)
                    {
                        //openCV image:?
                        Image<Bgr, Byte> currentFrame = new Image<Bgr, Byte>(bmp);  // return a OpenCv image
                        // Image<Bgr, Byte> currentFrame = ToOpenCVImage<Bgr, Byte>(bitmap);

                        //CV processing
                        Image<Gray, Byte> grayImage = currentFrame.Convert<Gray, Byte>().PyrDown().PyrUp();
                        Dest = new Image<Gray, Byte>(grayImage.Size);
                        CvInvoke.cvCanny(grayImage, Dest, 10, 60, 3);


                        // image.Source = ToBitmapSource(currentFrame);
                        image1.Source = ToBitmapSource(Dest);
                        edgesAv = true;
                        CalculateFps();
                    }



                }
                //Since we are coming from a triggered event, we are not expecting anything here, at least for this short tutorial


                else
                {
                    System.Diagnostics.Debug.WriteLine("depth bitmap empty :/");
                }
            }
        }

        /// <summary>
        /// ConvertDepthFrame:
        /// Converts the depth frame into its RGB version taking out all the player information and leaving only the depth.
        /// </summary>
        private byte[] ConvertDepthFrame(short[] depthFrame, DepthImageStream depthStream)
        {
            //System.Diagnostics.Debug.WriteLine("depthframe len :{0}", depthFrame.Length);
            //Run through the depth frame making the correlation between the two arrays
            for (int i16 = 0, i32 = 0; i16 < depthFrame.Length && i32 < this.depthFrame32.Length; i16++, i32 += 4) //4
            {
                //We don’t care about player’s information here, so we are just going to rule it out by shifting the value.
                int realDepth = depthFrame[i16] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                // System.Diagnostics.Debug.WriteLine("bitmaskwidth len :{0}", DepthImageFrame.PlayerIndexBitmaskWidth);
                //We are left with 13 bits of depth information that we need to convert into an 8 bit number for each pixel.
                //There are hundreds of ways to do this. This is just the simplest one.
                //Lets create a byte variable called Distance.
                //We will assign this variable a number that will come from the conversion of those 13 bits.
                byte Distance = 0;

                //XBox Kinects (default) are limited between 800mm and 4096mm.
                int MinimumDistance = 800;
                int MaximumDistance = 4096;

                //XBox Kinects (default) are not reliable closer to 800mm, so let’s take those useless measurements out.
                //If the distance on this pixel is bigger than 800mm, we will paint it in its equivalent gray
                if (realDepth > MinimumDistance)
                {
                    

                    //White = Close
                    //Black = Far
                    Distance = (byte)(255 - ((realDepth - MinimumDistance) * 255 / (MaximumDistance - MinimumDistance)));

                    //Use the distance to paint each layer (R G & B) of the current pixel.
                    //Painting R, G and B with the same color will make it go from black to gray
                    this.depthFrame32[i32 + RedIndex] = (byte)(Distance);
                    this.depthFrame32[i32 + GreenIndex] = (byte)(Distance);
                    this.depthFrame32[i32 + BlueIndex] = (byte)(Distance);
                }

                //If we are closer than 800mm, the just paint it red so we know this pixel is not giving a good value
                else
                {
                    this.depthFrame32[i32 + RedIndex] = 0;
                    this.depthFrame32[i32 + GreenIndex] = 150;
                    this.depthFrame32[i32 + BlueIndex] = 0;
                }
            }
            //Now that we are done painting the pixels, we can return the byte array to be painted
            return this.depthFrame32;
        }

   
        Bitmap ImageToBitmap(ColorImageFrame Image)
        {

            if (Image != null)
            {
                if (pixeldata == null)
                {
                    pixeldata = new byte[Image.PixelDataLength];
                }

                Image.CopyPixelDataTo(pixeldata);
                bmap = new Bitmap(Image.Width, Image.Height, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
                BitmapData bmapdata = bmap.LockBits(
                    new System.Drawing.Rectangle(0, 0, Image.Width, Image.Height),
                    ImageLockMode.WriteOnly,
                    bmap.PixelFormat);
                IntPtr ptr = bmapdata.Scan0;
                Marshal.Copy(pixeldata, 0, ptr, Image.PixelDataLength);
                bmap.UnlockBits(bmapdata);
                return bmap;
            }
            // byte[] pixeldata = new byte[Image.PixelDataLength];
            // Image.CopyPixelDataTo(pixeldata);
            else
            {
                return null;
            }
        }
        Bitmap DepthImageToBitmap(DepthImageFrame Image)
        {

            if (Image != null)
            {
                if (shortpixeldata == null)
                {
                    shortpixeldata = new short[Image.PixelDataLength];
                }

                Image.CopyPixelDataTo(shortpixeldata);
                bmap = new Bitmap(Image.Width, Image.Height, System.Drawing.Imaging.PixelFormat.Format16bppRgb565); //depth. 16bppGrayScale is the only format that works. We need to some how make this valid for OpenCV though. We don't get any data in any other format.
                if (bmap != null)
                {
                    System.Diagnostics.Debug.WriteLine("bmap created from new bitmap cmd");
                    /*MemoryStream ms = new MemoryStream(); MemoryStream ms = new MemoryStream();
                    bmap.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);
                    System.Windows.Media.Imaging.BitmapImage bImg = new System.Windows.Media.Imaging.BitmapImage();
                    bImg.BeginInit();
                    bImg.StreamSource = new MemoryStream(ms.ToArray());
                    bImg.EndInit();
                    //img is an Image control.
                    image.Source = bImg;
                    bmap.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);
                    System.Windows.Media.Imaging.BitmapImage bImg = new System.Windows.Media.Imaging.BitmapImage();
                    bImg.BeginInit();
                    bImg.StreamSource = new MemoryStream(ms.ToArray());
                    bImg.EndInit();
                    //img is an Image control.
                    image1.Source = bImg;*/
                    //Test code for display.
                    BitmapData bmapdata = bmap.LockBits(
                        new System.Drawing.Rectangle(0, 0, Image.Width, Image.Height),
                        ImageLockMode.WriteOnly,
                        bmap.PixelFormat);

                    System.Diagnostics.Debug.WriteLine("bmapdata created from bmap.lockbits cmd");
                    IntPtr ptr = bmapdata.Scan0;
                    System.Diagnostics.Debug.WriteLine("intptr created m scanned");
                    //IntPtr ptrdest = bmapdata.Scan0;
                    //CvInvoke.cvCvtScale(ptr,ptrdest, 0.5,0);
                    Marshal.Copy(shortpixeldata, 0, ptr, Image.PixelDataLength);
                    System.Diagnostics.Debug.WriteLine("marshal copy done");
                    bmap.UnlockBits(bmapdata);
                    System.Diagnostics.Debug.WriteLine("bmap unlocked");
                    if (bmap != null)
                        return bmap;
                    else
                        return null;
                }
                else
                    return null;
            }
            // byte[] pixeldata = new byte[Image.PixelDataLength];
            // Image.CopyPixelDataTo(pixeldata);
            else
            {
                return null;
            }
        }
        private Bitmap BitmapImage2Bitmap(BitmapImage bitmapImage)
        {
            // BitmapImage bitmapImage = new BitmapImage(new Uri("../Images/test.png", UriKind.Relative));

            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create(bitmapImage));
                enc.Save(outStream);
                System.Drawing.Bitmap bitmap = new System.Drawing.Bitmap(outStream);

                // return bitmap; <-- leads to problems, stream is closed/closing ...
                return new Bitmap(bitmap);
            }
        }

      



        public static BitmapSource ToBitmapSource(IImage image)
        {
            using (System.Drawing.Bitmap source = image.Bitmap)
            {
                IntPtr ptr = source.GetHbitmap(); //obtain the Hbitmap

                BitmapSource bs = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(
                     ptr,
                    IntPtr.Zero,
                    Int32Rect.Empty,
                    System.Windows.Media.Imaging.BitmapSizeOptions.FromEmptyOptions());
               
                int f = API.DeleteObject(ptr);

                return bs;
            }
        }
        private void WindowClosed(object sender, EventArgs e)
        {
            _kinectNui.Stop();
        }

        private void BtnStopClick(object sender, RoutedEventArgs e)
        {
            try
            {

                _kinectNui.Stop();
                btnStop.IsEnabled = false;
                btnStart.IsEnabled = true;
            }
            catch (NullReferenceException ex)
            {
                MessageBox.Show("Some error occurred. Please ensure that Kinect is connected properly.");
            }
            
        }

        private void BtnStartClick(object sender, RoutedEventArgs e)
        {

          
            InitializeNui();
            //btnStop.IsEnabled = true;
            //btnStart.IsEnabled = false;
        }

        private void BtnCameraUpClick(object sender, RoutedEventArgs e)
        {
            try
            {
                _kinectNui.ElevationAngle = _kinectNui.ElevationAngle + 5;
            }
            catch (InvalidOperationException ex)
            {
                MessageBox.Show(ex.Message);
            }
            catch (ArgumentOutOfRangeException outOfRangeException)
            {
                //Elevation angle must be between Elevation Minimum/Maximum"
                MessageBox.Show(outOfRangeException.Message);
            }
        }

        private void BtnCameraDownClick(object sender, RoutedEventArgs e)
        {
            try
            {
                _kinectNui.ElevationAngle = _kinectNui.ElevationAngle - 5;
            }
            catch (InvalidOperationException ex)
            {
                MessageBox.Show(ex.Message);
            }
            catch (ArgumentOutOfRangeException outOfRangeException)
            {
                //Elevation angle must be between Elevation Minimum/Maximum"
                MessageBox.Show(outOfRangeException.Message);
            }
        }

    }//end class
}
