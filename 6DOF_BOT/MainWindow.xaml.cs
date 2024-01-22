using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using System.Windows.Input;
using System.Windows.Media;
using System.IO;
using EasyModbus;
using System.IO.Ports;
//https://github.com/Gabryxx7/RobotArmHelix/tree/master
//https://github.com/biancheng1000/RAV2
//https://www.alanzucconi.com/2017/04/10/robotic-arms/
namespace _6DOF_BOT
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    class Nyjet {
        public Model3D modeli = null; //bartja e modeleve 3d per nyje
        public double kendi = 0;
        public double kendiMin = 0;
        public double kendiMax = 180;
        public double pikaerrotX = 0;
        public double pikaerrotY = 0;
        public double pikaerrotZ = 0;
        public int aksiirrotX = 0; //(1,0,0)per rrotullime rreth aksit x te pikes
        public int aksiirrotY = 0; //(0,1,0)per rrotullime rreth aksit x te pikes
        public int aksiirrotZ = 0; //(0,0,1)per rrotullime rreth aksit x te pikes

        public Nyjet(Model3D jmodel)
        {
            modeli = jmodel;
        }
    }
    class Vija
    {
        public LinesVisual3D vija;
        public Point3D pika_P;  //perfundimtare 
        public Point3D pika_F; // fillestare
        public Vija(LinesVisual3D vij)
        {
            vija = vij;
        }



    }

    public partial class MainWindow : Window
    {
        Model3DGroup KrahuRobotik = new Model3DGroup(); //Krahu rrobotik grupi i 3dmodeleve
        Model3DGroup KrahuRobotik_2dof = new Model3DGroup(); //Krahu rrobotik grupi i 3dmodeleve
        Model3D sfera = null;
        ModbusClient Roboti_Kom;
        MeshBuilder[] sferat_e_pozites_aktuale = new MeshBuilder [6];// ndryshimi nga v3
        GeometryModel3D[] gjeometria_e_sferve_te_pozites_aktuale=new GeometryModel3D[6];// ndryshimi nga v3
        ModelVisual3D[] paraqitja_e_sferave_ne_3d= new ModelVisual3D[6];// ndryshimi nga v3


        double L1 = 4, L2 = 4, Teta1 = 0.0, Teta2 = 0.0, x1 = 4.0, x2 = 8.0, y1 = 0.0, y2 = 0.0;
        int[] teta = new int[3];
        

        Vector3D reachingPoint = new Vector3D(0, 0, 0);
        List<Nyjet> nyjet = null;
        List<Nyjet> nyjet_2dof = null;
        bool ndrrimiinyjeve = false;
        bool duke_levizur = false;
        bool gjurmo_levizjet = false;
        Color ngjyra_e_vjeter = Colors.White;
        GeometryModel3D Modeli_i_vjeter_i_zgjedhur = null;
        string FollderiBaze = "";
        ModelVisual3D visual;
        double LearningRate = 0.01;
        double SamplingDistance = 0.15;
        double Kufiri_i_distances = 50;
        //provides render to model3d objects
        ModelVisual3D RoboticArm = new ModelVisual3D();
        ModelVisual3D RoboticArm_2dof = new ModelVisual3D();
        bool[] mberritja_e_vleres = new bool[6];
        int[] kendet_aktuale = { 0, 0, 0, 0 , 0 , 0  };
        Vija[] pozita_e_rrobotit = new Vija [6]; //LISTE ME 6 INSTANCA VIJA
        
        Transform3DGroup[] F = new Transform3DGroup[6];  //MODELET STL
        Transform3DGroup[] V = new Transform3DGroup[6];  // vIJAT
        Transform3DGroup[] P = new Transform3DGroup[6]; // PIKAT(Sferat)
        Transform3DGroup[] _2dof = new Transform3DGroup[3];


        RotateTransform3D R; //MATRICA RROTULLUESE
        TranslateTransform3D T; //MATRICA TRANSFORMUESE

        RotateTransform3D R_2dof; //MATRICA RROTULLUESE
        TranslateTransform3D T_2dof; //MATRICA TRANSFORMUESE
        Vector3D pika_e_arritur;
        int levizjet = 50;
        System.Windows.Forms.Timer timer1,timer2,timer3;
        




        private const string Nyja_1_3d = "nyja1v_1.0.stl";
        private const string Nyja_2_3d = "nyja2v_1.0.stl";
        private const string Nyja_3_3d = "nyja3v_1.0.stl";
        private const string Nyja_4_3d = "nyja4v_1.0.stl";
        private const string Nyja_5_3d = "nyja5v_1.0.stl";
        private const string Nyja_6_3d = "nyja6v_1.0.stl";
        private const string Baza_e_rrobotit = "bazav_1.0.stl";

        private const string Baza = "Baza_2dof.stl";
        private const string krahu1 = "krahu1_2dof.stl";
        private const string krahu2 = "krahu2_2dof.stl";




        public MainWindow()

        {
            InitializeComponent();
            z_nyjen.Value = 4;
            FollderiBaze = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + "\\3dModelet\\";
            List<string> emrat_e_3D_modeleve = new List<string>();
            
            emrat_e_3D_modeleve.Add(Nyja_1_3d);
            emrat_e_3D_modeleve.Add(Nyja_2_3d);
            emrat_e_3D_modeleve.Add(Nyja_3_3d);
            emrat_e_3D_modeleve.Add(Nyja_4_3d);
            emrat_e_3D_modeleve.Add(Nyja_5_3d);
            emrat_e_3D_modeleve.Add(Nyja_6_3d);
            emrat_e_3D_modeleve.Add(Baza_e_rrobotit);


            List<string> emrat_e_3D_modeleve_2dof = new List<string>();
            emrat_e_3D_modeleve_2dof.Add(Baza);
            emrat_e_3D_modeleve_2dof.Add(krahu1);
            emrat_e_3D_modeleve_2dof.Add(krahu2);

            RoboticArm.Content = Initialize_Environment(emrat_e_3D_modeleve);
            RoboticArm_2dof.Content = Initialize_Environment_2dof(emrat_e_3D_modeleve_2dof);

            //sfera per levizjet e nyjeve
            var nderto = new MeshBuilder(true, true);
            var pozita_ne_hapsire = new Point3D(0, 0, 0);
            nderto.AddSphere(pozita_ne_hapsire, 10, 15, 15);
            sfera = new GeometryModel3D(nderto.ToMesh(), Materials.Blue);
            visual = new ModelVisual3D();
            visual.Content = sfera;

            viewPort3d.Children.Add(visual);
            viewPort3d.Children.Add(RoboticArm);
            viewPort3d.Children.Add(RoboticArm_2dof);

            viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
            viewPort3d.Camera.LookDirection = new Vector3D(-658.952, 847.224, -549.795);
            viewPort3d.Camera.UpDirection = new Vector3D(0.250, -0.321, 0.914);
            viewPort3d.Camera.Position = new Point3D(540.624, -893.148, 628.976);
            //viewPort3d.Camera.Target??


            double[] kendet = { nyjet[0].kendi, nyjet[1].kendi, nyjet[2].kendi, nyjet[3].kendi, nyjet[4].kendi, nyjet[5].kendi };
            Kinematika_Direkte(kendet);
            ndrro_nyjen();
            timer1 = new System.Windows.Forms.Timer();
            timer1.Interval = 5;
            timer1.Tick += new System.EventHandler(timer1_tick);//disabled for further development

            timer2 = new System.Windows.Forms.Timer();
            timer2.Interval = 1; //shpejtesia
            timer2.Tick += new System.EventHandler(timer2_tick); //disabled for further development

            timer3 = new System.Windows.Forms.Timer();
            timer3.Interval=100;
            timer3.Tick += new System.EventHandler(timer3_tick);




        }

        private void Forward_kinematika()
        {
            x1 = L1 *Math.Cos(Teta1 * Math.PI / 180);
            y1 = L1 * Math.Sin(Teta1 * Math.PI / 180);

            x2 = x1 + L2 * Math.Cos((Teta1 + Teta2) * Math.PI / 180);
            y2 = y1 + L2 * Math.Sin((Teta1 + Teta2) * Math.PI / 180);

           
        }

    


        private void timer3_tick(object sender, EventArgs e)
        {
            int[] v_senzore = Roboti_Kom.ReadHoldingRegisters(0,7);
            int[] v_servo= Roboti_Kom.ReadHoldingRegisters(7, 4);
            int[] v_servo2 = Roboti_Kom.ReadHoldingRegisters(11,2);
            int[] moda = Roboti_Kom.ReadHoldingRegisters(13, 3);


            if (moda[0]==1 && moda[1]>=0 && moda[1] <= 3) {
                z_nyjen.Value = moda[1];
                if (moda[1]==0) { z_nyjen.Value = 6; }
                if (moda[1] == 1) { z_nyjen.Value = 0; }

            }
            else { z_nyjen.Value = 4; }


            for (byte i=0;i<4;i++)
            {
                nyjet[i].kendi = v_servo[i]-90;
            }


           
       //     nyjet[1].kendi = nyja_2.Value;
              nyjet[4].kendi= nyjet[5].kendi = 0;
         
            eg_kinematika_direkte();
            Kin_direkte_2dof();

            if (v_senzore[0] == 0) { S0.Fill = new SolidColorBrush(Color.FromRgb(255, 0, 0)); } else { S0.Fill = new SolidColorBrush(Color.FromRgb(0, 230, 0)); }
            if (v_senzore[1] == 0) { S1.Fill = new SolidColorBrush(Color.FromRgb(255, 0, 0)); } else { S1.Fill = new SolidColorBrush(Color.FromRgb(0, 230, 0)); }
            if (v_senzore[2] == 0) { S2.Fill = new SolidColorBrush(Color.FromRgb(255, 0, 0)); } else { S2.Fill = new SolidColorBrush(Color.FromRgb(0, 230, 0)); }
            if (v_senzore[3] == 0 ) { S4.Fill = new SolidColorBrush(Color.FromRgb(255, 0, 0)); } else { S4.Fill = new SolidColorBrush(Color.FromRgb(0, 230, 0)); }
            if (v_senzore[4] == 0 ) { S3.Fill = new SolidColorBrush(Color.FromRgb(255, 0, 0)); } else { S3.Fill = new SolidColorBrush(Color.FromRgb(0, 230, 0)); }
            if (v_senzore[5] == 0) { S5.Fill = new SolidColorBrush(Color.FromRgb(255, 0, 0)); } else { S5.Fill = new SolidColorBrush(Color.FromRgb(0, 230, 0)); }
            if (v_senzore[6] == 0) { S6.Fill = new SolidColorBrush(Color.FromRgb(255, 0, 0)); } else { S6.Fill = new SolidColorBrush(Color.FromRgb(0, 230, 0)); }

            teta[0]= v_servo2[0];
            teta[1]= v_servo2[1];
        }

        private void timer2_tick(object sender, EventArgs e)
        {
            Levizja_reale(kendet_aktuale); // me transformimin e modeleve 3d
            pr_bar.IsIndeterminate = true;

            for (int i = 0; i <= 5; i++)
            {
                if (kendet_aktuale[i] == nyjet[i].kendi) { mberritja_e_vleres[i] = true; }
                if (mberritja_e_vleres[0] && mberritja_e_vleres[1] && mberritja_e_vleres[2] && mberritja_e_vleres[3] && mberritja_e_vleres[4] && mberritja_e_vleres[5])
                {
                    for (int j = 0; j <= 5; j++)
                        mberritja_e_vleres[j] = false;

                    if (!gjurmo_levizjet) {
                        timer2.Stop();
                        pr_bar.IsIndeterminate = false;
                        pr_bar.Value = 100;

                    }
                    
                }
                if ((kendet_aktuale[i] != nyjet[i].kendi) && (kendet_aktuale[i] < nyjet[i].kendi)) { kendet_aktuale[i]++; }
                if ((kendet_aktuale[i] != nyjet[i].kendi) && (kendet_aktuale[i] > nyjet[i].kendi)) { kendet_aktuale[i]--; }
                


            }
            int[] kendet_e_konvertuara = new int[6];

            // kendet_e_konvertuara[j] = (kendet aktuale[j]>0) ?  (90-kendet aktuale[j]) :   (90+kendet aktuale[j]) ;
            for (int j = 0; j <= 5; j++)
            {

                if (j != 2 && j < 2) { kendet_e_konvertuara[j] = kendet_aktuale[j] + 90; } //    //per shkak te pozites fillestare te servos 3 ndryshimi nga v2
                if (j >= 2) { kendet_e_konvertuara[j] = (kendet_aktuale[j] >= 0) ? (90 - kendet_aktuale[j]) : (90 + Math.Abs(kendet_aktuale[j]));  } //kendet_e_konvertuara[j] = (kendet_aktuale[j] > 0) ? (90 + kendet_aktuale[j]) :  (90 + kendet_aktuale[j] );

            }
                //if (kendet_aktuale[j] > 0) { kendet_e_konvertuara[j] = kendet_aktuale[j] + 90; }
            
            Roboti_Kom.WriteMultipleRegisters(0, kendet_e_konvertuara);
            _kendet.Text = Convert.ToString( "N1:  "+kendet_e_konvertuara[0]+"   N2: "+ kendet_e_konvertuara[1]+ "   N3: " + kendet_e_konvertuara[2] + "   N4: " + kendet_e_konvertuara[3] + "   N5: " + kendet_e_konvertuara[4]+ "   N6: " + kendet_e_konvertuara[5]);
        }

        public void timer1_tick(object sender, EventArgs e)
        {
           
            double[] kendet = { nyjet[0].kendi, nyjet[1].kendi, nyjet[2].kendi, nyjet[3].kendi, nyjet[4].kendi, nyjet[5].kendi };
            kendet = Kinematika_Inverse(pika_e_arritur, kendet);
             nyjet[0].kendi = kendet[0];
            nyjet[1].kendi = kendet[1];
            nyjet[2].kendi = kendet[2];
            nyjet[3].kendi = kendet[3];
             nyjet[4].kendi = kendet[4];
             nyjet[5].kendi = kendet[5];

            if ((--levizjet) <= 0)
            {
             
                duke_levizur = false;
                timer1.Stop();
            }

        }

       public double[] Kinematika_Inverse(Vector3D qellimi, double[] kendet)
        {
           
            if (Distanca_nga_pika(qellimi, kendet) < Kufiri_i_distances)
            {
                levizjet = 0;
                return kendet;
            }

            double[] kendet_parardhese = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            kendet.CopyTo(kendet_parardhese, 0);
            for (int i = 0; i <= 5; i++)
            {
                
                double gradient = PartialGradient(qellimi, kendet, i);
                kendet[i] -= LearningRate * gradient;

                // Clamp
                kendet[i] = Clamp(kendet[i], nyjet[i].kendiMin, nyjet[i].kendiMax);

                // Early termination
                if (Distanca_nga_pika(qellimi, kendet) < Kufiri_i_distances || verteto_kendet(kendet_parardhese, kendet))
                {
                    levizjet = 0;
                    return kendet;
                }
            }

            return kendet;

        }

        public bool verteto_kendet(double[] kendet_e_vjetra, double[] kendet)
        {
            for (int i = 0; i <= 5; i++)
            {
                if (kendet_e_vjetra[i] != kendet[i])
                    return false;
            }

            return true;
        }

        public static T Clamp<T>(T vlera, T min, T max) //disabled for further developement
        where T : System.IComparable<T>
        {
            T rezultati = vlera;
            if (vlera.CompareTo(max) > 0)
                rezultati = max;
            if (vlera.CompareTo(min) < 0)
                rezultati = min;
            return rezultati;
        }

        public double PartialGradient(Vector3D qellimi, double[] kendet, int i) //inverse kinematics using partial gradient
        {
           
            double kendi = kendet[i];


            double funksioni_f_x = Distanca_nga_pika(qellimi,kendet);

            kendet[i] += SamplingDistance;

            double funksioni_f_x_distanca = Distanca_nga_pika(qellimi, kendet);
            double gradient = (funksioni_f_x_distanca - funksioni_f_x) / SamplingDistance;

            kendet[i] = kendi;

            return gradient;
        }

        public double Distanca_nga_pika(Vector3D qellimi, double[] kendet)
        {
            
            Vector3D pika = Kinematika_Direkte(kendet);
            return Math.Sqrt(Math.Pow((pika.X - qellimi.X), 2.0) + Math.Pow((pika.Y - qellimi.Y), 2.0) + Math.Pow((pika.Z - qellimi.Z), 2.0));
        }

        private Model3DGroup Initialize_Environment(List<string> emrat_e_3D_modeleve)
        {
            try
            {

                ModelImporter import = new ModelImporter();
                nyjet = new List<Nyjet>();

                foreach (string emri_i_modelit in emrat_e_3D_modeleve)
                {
                    var materialGroup = new MaterialGroup();
                    Color mainColor = Colors.White;
                    EmissiveMaterial emissMat = new EmissiveMaterial(new SolidColorBrush(mainColor));
                    DiffuseMaterial diffMat = new DiffuseMaterial(new SolidColorBrush(mainColor));
                    SpecularMaterial specMat = new SpecularMaterial(new SolidColorBrush(mainColor), 200);
                    materialGroup.Children.Add(emissMat);
                    materialGroup.Children.Add(diffMat);
                    materialGroup.Children.Add(specMat);

                    var link = import.Load(FollderiBaze + emri_i_modelit);
                    GeometryModel3D model = link.Children[0] as GeometryModel3D;
                    model.Material = materialGroup;
                    model.BackMaterial = materialGroup;
                    nyjet.Add(new Nyjet(link));
                }


                KrahuRobotik.Children.Add(nyjet[0].modeli);
                KrahuRobotik.Children.Add(nyjet[1].modeli);
                KrahuRobotik.Children.Add(nyjet[2].modeli);
                KrahuRobotik.Children.Add(nyjet[3].modeli);
                KrahuRobotik.Children.Add(nyjet[4].modeli);
                KrahuRobotik.Children.Add(nyjet[5].modeli);
                KrahuRobotik.Children.Add(nyjet[6].modeli);


                nyjet[0].kendiMin = -90;
                nyjet[0].kendiMax = 90;
                nyjet[0].aksiirrotX = 0;
                nyjet[0].aksiirrotY = 0;
                nyjet[0].aksiirrotZ = 1;
                nyjet[0].pikaerrotX = 21.9996515679443;
                nyjet[0].pikaerrotY = -0.1;
                nyjet[0].pikaerrotZ = 60;

                nyjet[1].kendiMin = -90;
                nyjet[1].kendiMax = 90;
                nyjet[1].aksiirrotX = 0;
                nyjet[1].aksiirrotY = 1;
                nyjet[1].aksiirrotZ = 0;
                nyjet[1].pikaerrotX = 22.5384436701509;
                nyjet[1].pikaerrotY = 2.29616724738579;
                nyjet[1].pikaerrotZ = 117.043670150987;

                nyjet[2].kendiMin = -90;
                nyjet[2].kendiMax = 90;
                nyjet[2].aksiirrotX = 0;
                nyjet[2].aksiirrotY = 1;
                nyjet[2].aksiirrotZ = 0;
                nyjet[2].pikaerrotX = 22.3716608594659;
                nyjet[2].pikaerrotY = -6;
                nyjet[2].pikaerrotZ = 217.9216027;

                nyjet[3].kendiMin = -90;
                nyjet[3].kendiMax = 90;
                nyjet[3].aksiirrotX = 0;
                nyjet[3].aksiirrotY = 1;
                nyjet[3].aksiirrotZ = 0;
                nyjet[3].pikaerrotX = 21.3006968641122;
                nyjet[3].pikaerrotY = 12;
                nyjet[3].pikaerrotZ = 314.953426248548;

                nyjet[4].kendiMin = -90;
                nyjet[4].kendiMax = 90;
                nyjet[4].aksiirrotX = 0;
                nyjet[4].aksiirrotY = 1;
                nyjet[4].aksiirrotZ = 0;
                nyjet[4].pikaerrotX = 25;
                nyjet[4].pikaerrotY = 0;
                nyjet[4].pikaerrotZ = 415;

                nyjet[5].kendiMin = -90;
                nyjet[5].kendiMax = 90;
                nyjet[5].aksiirrotX = 1;
                nyjet[5].aksiirrotY = 0;
                nyjet[5].aksiirrotZ = 0;
                nyjet[5].pikaerrotX = 30;
                nyjet[5].pikaerrotY = 0;
                nyjet[5].pikaerrotZ = 514;


            }
            catch (Exception E)
            {
                MessageBox.Show("Exception Error:" + E.StackTrace);
            }
            return KrahuRobotik;
        }

        private Model3DGroup Initialize_Environment_2dof(List<string> emrat_e_3D_modeleve_2dof)
        {
            try
            {

                ModelImporter import = new ModelImporter();
                nyjet_2dof = new List<Nyjet>();

                foreach (string emri_i_modelit in emrat_e_3D_modeleve_2dof)
                {
                    var materialGroup = new MaterialGroup();
                    Color mainColor = Colors.White;
                    EmissiveMaterial emissMat = new EmissiveMaterial(new SolidColorBrush(mainColor));
                    DiffuseMaterial diffMat = new DiffuseMaterial(new SolidColorBrush(mainColor));
                    SpecularMaterial specMat = new SpecularMaterial(new SolidColorBrush(mainColor), 200);
                    materialGroup.Children.Add(emissMat);
                    materialGroup.Children.Add(diffMat);
                    materialGroup.Children.Add(specMat);

                    var link = import.Load(FollderiBaze + emri_i_modelit);
                    GeometryModel3D model = link.Children[0] as GeometryModel3D;
                    model.Material = materialGroup;
                    model.BackMaterial = materialGroup;
                    nyjet_2dof.Add(new Nyjet(link));
                }


                KrahuRobotik_2dof.Children.Add(nyjet_2dof[0].modeli);
                KrahuRobotik_2dof.Children.Add(nyjet_2dof[1].modeli);
                KrahuRobotik_2dof.Children.Add(nyjet_2dof[2].modeli);
                


                nyjet_2dof[0].kendiMin = -90;
                nyjet_2dof[0].kendiMax = 90;
                nyjet_2dof[0].aksiirrotX = 0;
                nyjet_2dof[0].aksiirrotY = 0;
                nyjet_2dof[0].aksiirrotZ = 1;
                nyjet_2dof[0].pikaerrotX = -10.2;
                nyjet_2dof[0].pikaerrotY = -325.195121951223;
                nyjet_2dof[0].pikaerrotZ = 49.6794425087109;
               
                nyjet_2dof[1].kendiMin = -90;
                nyjet_2dof[1].kendiMax = 90;
                nyjet_2dof[1].aksiirrotX = 0;
                nyjet_2dof[1].aksiirrotY = 0;
                nyjet_2dof[1].aksiirrotZ = 1;
                nyjet_2dof[1].pikaerrotX = -10.6198606271775;
                nyjet_2dof[1].pikaerrotY = -326.114982578399;
                nyjet_2dof[1].pikaerrotZ = 62.4390243902438;
                
                nyjet_2dof[2].kendiMin = -90;
                nyjet_2dof[2].kendiMax = 90;
                nyjet_2dof[2].aksiirrotX = 0;
                nyjet_2dof[2].aksiirrotY = 0;
                nyjet_2dof[2].aksiirrotZ = 1;
                nyjet_2dof[2].pikaerrotX = -86.2787456445986;
                nyjet_2dof[2].pikaerrotY = -326.114982578399;
                nyjet_2dof[2].pikaerrotZ = 81.7986062717774;

               


            }
            catch (Exception E)
            {
                MessageBox.Show("Exception Error:" + E.StackTrace);
            }
            return KrahuRobotik_2dof;
        }



        private Color ndrro_ngjyten_e_modelit(Nyjet jnyjet, Color ngjyra_e_re)
        {
            Model3DGroup modelet = ((Model3DGroup)jnyjet.modeli);
            return ndrro_ngjyten_e_modelit(modelet.Children[0] as GeometryModel3D, ngjyra_e_re);
        }

        private Color ndrro_ngjyten_e_modelit(GeometryModel3D pModeli, Color ngjyra_e_re)
        {


            if (pModeli == null)
                return ngjyra_e_vjeter;

            Color previousColor = Colors.Black;

            MaterialGroup mg = (MaterialGroup)pModeli.Material;
            if (mg.Children.Count > 0)
            {
                try
                {
                    previousColor = ((EmissiveMaterial)mg.Children[0]).Color;
                    ((EmissiveMaterial)mg.Children[0]).Color = ngjyra_e_re;
                    ((DiffuseMaterial)mg.Children[1]).Color = ngjyra_e_re;
                }
                catch (Exception)
                {
                    previousColor = ngjyra_e_re;
                }
            }

            return previousColor;
        }

        private void zgjedhModelin(Model3D pModeli)
        {
            try
            {
                Model3DGroup modelet = ((Model3DGroup)pModeli);
                Modeli_i_vjeter_i_zgjedhur = modelet.Children[0] as GeometryModel3D;
            }
            catch (Exception)
            {
                Modeli_i_vjeter_i_zgjedhur = (GeometryModel3D)pModeli;
            }
            ngjyra_e_vjeter = ndrro_ngjyten_e_modelit(Modeli_i_vjeter_i_zgjedhur, ColorHelper.HexToColor("#5A4AFF"));
        }


        private void viewPort3d_MouzgjeftButtonUp(object sender, MouseButtonEventArgs e)
        {

        }

        private void viewPort3d_MouzgjeftButtonDown(object sender, MouseButtonEventArgs e)
        {

        }

        private void pika_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (ndrrimiinyjeve)
                return;

            int zgj = ((int)z_nyjen.Value) - 1;
            nyjet[zgj].pikaerrotX = pikaX.Value;
            nyjet[zgj].pikaerrotY = pikaY.Value;
            nyjet[zgj].pikaerrotZ = pikaZ.Value;
            pozita_e_sferes();
        }

        private void z_nyjen_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ndrro_nyjen();
        }

        private void unzgjektoModelin()
        {
            ndrro_ngjyten_e_modelit(Modeli_i_vjeter_i_zgjedhur, ngjyra_e_vjeter);
        }

        private void pozita_e_sferes()
        {
            int zgj = ((int)z_nyjen.Value) - 1;
            if (zgj < 0)
                return;

            Transform3DGroup F = new Transform3DGroup();
            F.Children.Add(new TranslateTransform3D(nyjet[zgj].pikaerrotX, nyjet[zgj].pikaerrotY, nyjet[zgj].pikaerrotZ));
            F.Children.Add(nyjet[zgj].modeli.Transform);
            sfera.Transform = F;
        }

        private void ndrro_nyjen()
        {
            if (nyjet == null)
                return;
            int zgj = ((int)z_nyjen.Value) ;

           
            ndrrimiinyjeve = true;
            unzgjektoModelin();
            if (zgj < 0)
            {
                pikaX.IsEnabled = false;
                pikaY.IsEnabled = false;
                pikaZ.IsEnabled = false;
                aksiX.IsEnabled = false;
                aksiY.IsEnabled = false;
                aksiZ.IsEnabled = false;
            }
            else
            {
                if (!pikaX.IsEnabled)
                {
                    pikaX.IsEnabled = true;
                    pikaY.IsEnabled = true;
                    pikaZ.IsEnabled = true;
                    aksiX.IsEnabled = true;
                    aksiY.IsEnabled = true;
                    aksiZ.IsEnabled = true;
                }
                pikaX.Value = nyjet[zgj].pikaerrotX;
                pikaY.Value = nyjet[zgj].pikaerrotY;
                pikaZ.Value = nyjet[zgj].pikaerrotZ;
                aksiX.IsChecked = nyjet[zgj].aksiirrotX == 1 ? true : false;
                aksiY.IsChecked = nyjet[zgj].aksiirrotY == 1 ? true : false;
                aksiZ.IsChecked = nyjet[zgj].aksiirrotZ == 1 ? true : false;
                zgjedhModelin(nyjet[zgj].modeli);
                pozita_e_sferes();
            }
            ndrrimiinyjeve = false;
        }

   /*     private void nyja_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (duke_levizur)
                return;
            try
            {
                nyjet[0].kendi = nyja_1.Value;
                nyjet[1].kendi = nyja_2.Value;
                nyjet[2].kendi = nyja_3.Value;
                nyjet[3].kendi = nyja_4.Value;
                nyjet[4].kendi = nyja_5.Value;
                nyjet[5].kendi = nyja_6.Value;
                eg_kinematika_direkte();
            }
            catch (Exception)
            {

            }       
        }*/

        private void eg_kinematika_direkte()
        {

            double[] kendet = { nyjet[0].kendi, nyjet[1].kendi, nyjet[2].kendi, nyjet[3].kendi, nyjet[4].kendi, nyjet[5].kendi };
            Kinematika_Direkte(kendet);
            pozita_e_sferes();
        }

        public Vector3D Kinematika_Direkte(double[] kendet)
        {

            for (int i = 0; i <= 5; i++)
            {
                if (i == 0)
                {
                    F[i] = new Transform3DGroup();
                    R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(nyjet[i].aksiirrotX, nyjet[i].aksiirrotY, nyjet[i].aksiirrotZ), kendet[i]), new Point3D(nyjet[i].pikaerrotX, nyjet[i].pikaerrotY, nyjet[i].pikaerrotZ));
                    F[i].Children.Add(R);
                }
                else
                {
                    F[i] = new Transform3DGroup();
                    T = new TranslateTransform3D(0, 0, 0);
                    R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(nyjet[i].aksiirrotX, nyjet[i].aksiirrotY, nyjet[i].aksiirrotZ), kendet[i]), new Point3D(nyjet[i].pikaerrotX, nyjet[i].pikaerrotY, nyjet[i].pikaerrotZ));
                    F[i].Children.Add(T);
                    F[i].Children.Add(R);
                    F[i].Children.Add(F[i-1]);
                }        
            }

            for (int i = 0; i <= 5; i++)
            {
                nyjet[i].modeli.Transform = F[i];
            }

            return new Vector3D(nyjet[5].modeli.Bounds.Location.X, nyjet[5].modeli.Bounds.Location.Y, nyjet[5].modeli.Bounds.Location.Z);

        }

        public void pozita_Click(object sender, RoutedEventArgs e)
        {
            if (timer1.Enabled)
            {
               
                duke_levizur = false;
                timer1.Stop();
                levizjet = 0;
            }
            else
            {
                sfera.Transform = new TranslateTransform3D(pika_e_arritur);
                levizjet = 5000;
                duke_levizur = true;
                timer1.Start();
            }
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            
            try
            {
                Roboti_Kom = new ModbusClient(com_porta.Text);
                Roboti_Kom.Parity = System.IO.Ports.Parity.None;
                Roboti_Kom.Baudrate = int.Parse(_baudrate.Text);
                Roboti_Kom.UnitIdentifier = Convert.ToByte(s_id.Value);
                
                Roboti_Kom.Connect();
                lidhu_but.IsEnabled = false;



               // Hapsira_me_vija();
               // Sferat_levizese();





                levizja.IsEnabled = true;
                Shkyqu.IsEnabled = true;
                lidhu_but.IsEnabled = false;
                Rrb_gjendja.SelectAll();
                Rrb_gjendja.Selection.Text = "";
                Rrb_gjendja.AppendText("Connected ");
                Rrb_gjendja.AppendText("\nReturn to Normal state");
                timer3.Start();
            }
            catch (Exception err )
            {
                Rrb_gjendja.SelectAll();
                Rrb_gjendja.Selection.Text = "";         
                MessageBox.Show("Gabim ne lidhje njeri nga parametrat eshte dhene gabim provo perseri", "Informacion mbi Lidhjen", MessageBoxButton.OK, MessageBoxImage.Information);
                Rrb_gjendja.AppendText("\n Provo perseri Kontrollo te gjithe parametrat gabimi eshte  -----> \n");
                Rrb_gjendja.AppendText( Convert.ToString(err));
                Rrb_gjendja.AppendText("<-------");

            }
        }

        private void com_porta_DropDownOpened(object sender, EventArgs e)
        {
            String[] portet = SerialPort.GetPortNames();//referenca per io.
            com_porta.Items.Clear();
            foreach (string comport in portet)
            {
                com_porta.Items.Add(comport);
            }
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            int[] kendet_aktuale = { 0, 0, 0, 0 };
                      
            Roboti_Kom.Disconnect();
            timer3.Stop();
            levizja.IsEnabled = false;
            Shkyqu.IsEnabled = false;
            lidhu_but.IsEnabled = true;
            Rrb_gjendja.SelectAll();
            Rrb_gjendja.Selection.Text = "";
            Rrb_gjendja.AppendText("\nShkyqur");

        }

       

        public void Levizja_reale(int[] kendet_aktuale)
        {
            for (int i = 0; i <= 3; i++)
            {
                if (i == 0)
                {
                    V[i] = new Transform3DGroup();
                    R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(nyjet[i].aksiirrotX, nyjet[i].aksiirrotY, nyjet[i].aksiirrotZ), kendet_aktuale[i]), new Point3D(nyjet[i].pikaerrotX, nyjet[i].pikaerrotY, nyjet[i].pikaerrotZ));
                    V[i].Children.Add(R);
                }
                else
                {
                    V[i] = new Transform3DGroup();

                    R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(nyjet[i].aksiirrotX, nyjet[i].aksiirrotY, nyjet[i].aksiirrotZ), kendet_aktuale[i]), new Point3D(nyjet[i].pikaerrotX, nyjet[i].pikaerrotY, nyjet[i].pikaerrotZ));
                    // F2.Children.Add(T);
                    V[i].Children.Add(R);
                    V[i].Children.Add(V[i-1]);
                }

            }

            for (int i = 0; i <= 5; i++)
            {
                if (i < 3) { pozita_e_rrobotit[i].vija.Transform = V[i]; paraqitja_e_sferave_ne_3d[i].Transform = V[i]; } else
                {
                    pozita_e_rrobotit[i].vija.Transform = V[3];
                    paraqitja_e_sferave_ne_3d[i].Transform = V[3];
                }
                
            }

        }

        private void _levizja_Click(object sender, RoutedEventArgs e)
        {
            timer2.Start();
        }

        public void Hapsira_me_vija()
        {
            for (int i = 0; i <= 5; i++)
            {
                pozita_e_rrobotit[i] = new Vija(new LinesVisual3D()); //zgjedhja
                switch (i)
                {
                    case 0:
                        pozita_e_rrobotit[0].vija.Color = Color.FromRgb(191, 0, 255);
                        break;
                    case 1:
                        pozita_e_rrobotit[1].vija.Color = Color.FromRgb(255, 255, 0);
                        break;
                    case 2:
                        pozita_e_rrobotit[2].vija.Color = Color.FromRgb(0, 255, 0);
                        break;
                    case 3:
                        pozita_e_rrobotit[3].vija.Color = Color.FromRgb(255, 0, 0); ;
                        break;
                    case 4:
                        pozita_e_rrobotit[4].vija.Color = Color.FromRgb(255, 0, 0);
                        break;
                    case 5:
                        pozita_e_rrobotit[5].vija.Color = Color.FromRgb(255, 0, 0);
                        break;

                    default:

                        break;
                }

                if (i <= 5)
                {
                    if (i != 3 && i < 3)
                    {
                        pozita_e_rrobotit[i].pika_F = new Point3D(nyjet[0].pikaerrotX, 0, nyjet[i].pikaerrotZ);
                        pozita_e_rrobotit[i].pika_P = new Point3D(nyjet[0].pikaerrotX, 0, nyjet[i + 1].pikaerrotZ);

                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_F);
                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_P);
                    }

                    if (i == 3)
                    {
                        pozita_e_rrobotit[i].pika_F = new Point3D(nyjet[0].pikaerrotX, 0, nyjet[i].pikaerrotZ);
                        pozita_e_rrobotit[i].pika_P = new Point3D(nyjet[0].pikaerrotX, 0, nyjet[i].pikaerrotZ + 10);
                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_F);
                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_P);
                    }
                    if (i == 4)
                    {
                        pozita_e_rrobotit[i].pika_F = new Point3D(pozita_e_rrobotit[3].pika_P.X, pozita_e_rrobotit[3].pika_P.Y, pozita_e_rrobotit[3].pika_P.Z);
                        pozita_e_rrobotit[i].pika_P = new Point3D(pozita_e_rrobotit[3].pika_P.X + 30, pozita_e_rrobotit[3].pika_P.Y, pozita_e_rrobotit[3].pika_P.Z);
                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_F);
                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_P);
                    }
                    if (i == 5)
                    {
                        pozita_e_rrobotit[i].pika_F = new Point3D(pozita_e_rrobotit[4].pika_P.X, pozita_e_rrobotit[4].pika_P.Y, pozita_e_rrobotit[4].pika_P.Z);
                        pozita_e_rrobotit[i].pika_P = new Point3D(pozita_e_rrobotit[4].pika_P.X, pozita_e_rrobotit[4].pika_P.Y, pozita_e_rrobotit[4].pika_P.Z + 40);
                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_F);
                        pozita_e_rrobotit[i].vija.Points.Add(pozita_e_rrobotit[i].pika_P);
                    }
                    viewPort3d.Children.Add(pozita_e_rrobotit[i].vija);
                }
                


            }
        }

        

        private void CheckBox_Checked(object sender, RoutedEventArgs e)
        {
            gjurmo_levizjet=true;
            komunikimi.IsEnabled = false;
        }

        private void gjurmo_lev_Unchecked(object sender, RoutedEventArgs e)
        {
            gjurmo_levizjet = false;
            komunikimi.IsEnabled = true;
        }

        private void Kin_direkte_2dof()
        {
            for (byte i=0;i<3;i++) {
                if (i==0)
                {
                    _2dof[i] = new Transform3DGroup();
                    R_2dof = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(nyjet_2dof[i].aksiirrotX, nyjet_2dof[i+1].aksiirrotY, nyjet_2dof[i].aksiirrotZ), teta[i]), new Point3D(nyjet_2dof[i].pikaerrotX, nyjet_2dof[i].pikaerrotY, nyjet_2dof[i].pikaerrotZ));
                    _2dof[i].Children.Add(R_2dof);
                }
                else
                {
                    _2dof[i] = new Transform3DGroup();
                    T_2dof = new TranslateTransform3D(0, 0, 0);
                    R_2dof = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(nyjet_2dof[i].aksiirrotX, nyjet_2dof[i].aksiirrotY, nyjet_2dof[i].aksiirrotZ), teta[i-1]), new Point3D(nyjet_2dof[i].pikaerrotX, nyjet_2dof[i].pikaerrotY, nyjet_2dof[i].pikaerrotZ));
                    _2dof[i].Children.Add(T_2dof);
                    _2dof[i].Children.Add(R_2dof);
                    _2dof[i].Children.Add(_2dof[i-1]);
                }
                
                
            }
            

            for (byte i = 1; i < 3; i++)
            {
                nyjet_2dof[i].modeli.Transform= _2dof[i];
                    }
        }

        private void Sferat_levizese() { //sferat per qendrat rrotulluese 

            for (int i = 0; i <= 5; i++) {
                sferat_e_pozites_aktuale[i] = new MeshBuilder(true, true);
                sferat_e_pozites_aktuale[i].AddSphere(pozita_e_rrobotit[i].pika_F,5,15,15);   //kendet_aktuale
                gjeometria_e_sferve_te_pozites_aktuale[i]= new GeometryModel3D(sferat_e_pozites_aktuale[i].ToMesh(), Materials.Gold);
                paraqitja_e_sferave_ne_3d[i]=new ModelVisual3D();
                paraqitja_e_sferave_ne_3d[i].Content = gjeometria_e_sferve_te_pozites_aktuale[i];
                viewPort3d.Children.Add(paraqitja_e_sferave_ne_3d[i]);
                //shtoj ne viewport ndryshimi nga v3
            }

/*
           var nderto = new MeshBuilder(true, true);
            var pozita_ne_hapsire = new Point3D(0, 0, 0);
            nderto.AddSphere(pozita_ne_hapsire, 10, 15, 15);
            sfera = new GeometryModel3D(nderto.ToMesh(), Materials.Blue);
            visual = new ModelVisual3D();
            visual.Content = sfera;
*/

        }   

    }

    }   

