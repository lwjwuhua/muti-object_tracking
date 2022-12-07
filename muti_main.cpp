// muti-object_tracking.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include<iterator>
#include<algorithm>
#include "Dense"
#include "measurement_package.h"
#include"measurement_package.h"
// #include "tracking.h"
// #include"association.h"
#include <cmath>
#include"track_list_manage.h"
#include"kf.h"
#include"association.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::istringstream;
using std::string;
using std::vector;
using std::unique;

#include <iostream>

int main()
{
    bool issvd;
    /*导入数据*/

    vector <MeasurementPackage> measurement_pack_list;

     vector < VectorXd > meas_list;  //轨迹列表
    
    Track_manager track_manager ;
    Track_list_manag  meas__;
    KalmanFilter kf;
    PDA pda;
    // hardcoded input file with laser and radar measurements
  //  string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
    string in_file_name_ = "1.txt";

    string out_file_name_ = "1234-output.txt";

    ifstream in_file(in_file_name_.c_str(), ifstream::in);
    ofstream out_file(out_file_name_.c_str(), ofstream::out);

    if (!in_file.is_open()) {
        cout << "Cannot open input file: " << in_file_name_ << endl;
    }

    string line;
    // set i to get only first 3 measurments



    int i = 0;

    /* step1:  数据加载读取到measuremen_package   */

    while (getline(in_file, line) && (i <= 500)) {

        MeasurementPackage meas_package;

        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type; // reads first element from the current line
        int64_t timestamp;
        float a_;
        if (sensor_type.compare("L") == 0) {  // laser measurement
          // read measurements
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            float z;
            iss >> x;
            iss >> y;

            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

            measurement_pack_list.push_back(meas_package);


        }
        else if (sensor_type.compare("R") == 0) {
            // Skip Radar measurements
            continue;
        }


        ++i;

    }

    MatrixXd p(4, 4);
    p << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    size_t N = measurement_pack_list.size();
    int count_t = 0;
    
    int j = 0;
   

    for (size_t k = 0; k < N; ++k)
    {


        
        int i = k;
        
        int e = 0;
        int count = 0;

        
      
        /*      遍历轨迹列表，逐个进行kalman预测    */
        cout << count_t << "时刻的kalman_predict" << endl;
        int s_ = track_manager.trackwith_id.size();
       if (count_t!=0)
      {
          
          for (size_t i = 0; i <s_; i++)
          {
              cout << i << "测量的kaman预测" << endl;
              kf.Predict(count_t, track_manager.trackwith_id[i].sig_track_state_list, track_manager.trackwith_id[i].sig_track_cov_list);

              track_manager.trackwith_id[i].sig_track_state_list = kf.pre_x_;
              track_manager.trackwith_id[i].sig_track_cov_list = kf.pre_P_;
          }
      }
    
       

        /*  获取t时刻的所有量测-  */
        cout << count_t << "时刻的所有量测---------开始-----------" << endl;
        meas_list.clear();
        for (i; (e < 100) && (i < N); i++)
        {
           

            if (i != 0) {
                e = measurement_pack_list[i].timestamp_ - measurement_pack_list[i - 1].timestamp_;
                cout << "mea_pack "<<i<<"==" << measurement_pack_list[i - 1].raw_measurements_ << endl;
                meas_list.push_back(measurement_pack_list[i-1].raw_measurements_);

                ///* ------------ 添加轨迹列表 ,并设置id---------------  */ 
                if ( count_t==0 )
                {
                    track_manager.Addtrack_list(count, measurement_pack_list[i - 1].raw_measurements_, p);
                }
                
                
            }

            cout << "--------------------" << i << "= i" << "------------------" << endl;

            count++;
        }

        cout << count_t << "时刻-------------------------量测结束-----------------————--" << endl;



        /*航迹起始，，暂时跳过 */

        /*  查看该时刻添加进轨迹列表的元素， */
       //  track_manager.trackwith_id.erase(std::begin(track_manager.trackwith_id) + 2);

        //int s = track_manager.trackwith_id.size();
        //for (size_t i = 0; i < s; i++)
        //{
        //    cout << "meas_list_track_id" << " == " << track_manager.trackwith_id[i].id << endl;
        //    cout << "meas_list_track_state" << " == " << track_manager.trackwith_id[i].sig_track_state_list << endl;

        //}
       


   
        int N = track_manager.trackwith_id.size();
        int M = meas_list.size();
        MatrixXd assosia_matri(N,M); //设置关联矩阵

        for (size_t i = 0; i < N; i++)
     {
             for (size_t j = 0; j < M; j++)
        {
              assosia_matri(i, j) = 0x3f3f3f3f;
          //  assosia_matri(i, j) = 0;
        }
     }



       
        /*      用量测来对每个轨迹进行数据关联       */
        if (count_t !=0)
        {
           
         
          
            for (size_t k = 0; k < N; k++)
            {
                cout << k << "轨迹的数据关联" << endl;

                for (size_t m_ = 0; m_ < M; m_++)
                {
                     pda.Association(k, m_, meas_list[m_], track_manager.trackwith_id[k].sig_track_state_list, track_manager.trackwith_id[k].sig_track_cov_list);
                     
                     if (pda.isassocia)
                     {
                         assosia_matri(k, m_) = pda.distance;
                        
                     }
                    

                }
            }



            /*  int s = distac_match.size();
       for (size_t i___=0; i___ <s; i___++)
       {
           cout << "====================================gatemach==========" << distac_match[i___] << endl;
       }*/


       //   cout  << assosia_matri << endl;

            vector<int>index;  // 存储需要update的轨迹与测量索引值
            vector<int>indey;
            index.clear();
            indey.clear();

            /*----在关联矩阵中找马氏距离最近的元素的索引------*/

            issvd = true;
            while (issvd)
            {
                pda.Findlatest_element(N, M, assosia_matri);

                /*cout << "indx =====" << pda.indx << endl;
                cout << "indy====" << pda.indy << endl;

                cout << "min ====" << assosia_matri(pda.indx, pda.indy) << endl;*/

                index.push_back(pda.indx);
                indey.push_back(pda.indy);




                for (size_t k_ = 0; k_ < N; k_++)
                {
                    assosia_matri(k_, pda.indy) = 0x3f3f3f3f;
                }
                for (size_t k__ = 0; k__ < M; k__++)
                {
                    assosia_matri(pda.indx, k__) = 0x3f3f3f3f;
                }


                int z = assosia_matri.jacobiSvd().rank();

                if (z == 1)
                {
                    issvd = false;
                }


            }

            pda.Findlatest_element(N, M, assosia_matri);
           /*  cout << "indx =====" << pda.indx << endl;
             cout << "indy====" << pda.indy << endl;*/

            if (  assosia_matri(pda.indx, pda.indy)!= 0x3f3f3f3f)
            {
                cout << "min ====" << assosia_matri(pda.indx, pda.indy) << endl;
                assosia_matri(pda.indx, pda.indy) = 0x3f3f3f3f;
                index.push_back(pda.indx);
                indey.push_back(pda.indy);
            }
           




            index.erase(unique(index.begin(), index.end()), index.end());
            indey.erase(unique(indey.begin(), indey.end()), indey.end());

             for (size_t x = 0; x < index.size(); x++)
             {
                 cout << "index-----========" << index[x] << endl;
                 cout << "indey-----========" << indey[x] << endl;
             }
             


             /* 找出剩下未匹配的轨迹——测量 */
             vector<int> temp_t(N);
             for (size_t nt = 0; nt < N; nt++)
             {
                 temp_t[nt] = nt;
             }
             vector<int>temp_m(M);
             for (size_t mt = 0; mt < M; mt++)
             {
                 temp_m[mt] = mt;
             }

           
            vector<int> unsign_match_track_id;
            vector<VectorXd> unsign_match_meas;
            unsign_match_track_id.clear();
            unsign_match_meas.clear();

          
            for (size_t tr = 0; tr < track_manager.trackwith_id.size(); tr++)
            {
                for (size_t tri = 0; tri < index.size(); tri++)
                {
                    if (tr==index[tri])
                    {
                        temp_t[tr] = N;
                    }
                }
            }

            for (size_t nt_ = 0; nt_ < N; nt_++)
            {
                if (temp_t[nt_]!=N)
                {
                    unsign_match_track_id.push_back(nt_); 
                }
            }


            for (size_t mr = 0; mr < meas_list.size(); mr++)
            {
                for (size_t mri = 0; mri < indey.size(); mri++)
                {
                    if (mr == indey[mri])
                    {
                        temp_m[mr] = M;
                    }
                }
            }


            for (size_t mt_ = 0; mt_ < M; mt_++)
            {
                if (temp_m[mt_] != M)
                {
                    unsign_match_meas.push_back(meas_list[mt_]);
                }
            }


            /* kf--update */
            for (size_t k_ = 0; k_ < index.size(); k_++)
            {
                cout << "轨迹" << index[k_] <<"与测量"<<indey[k_] << "===================update======================" << endl;
                kf.Update1(meas_list[indey[k_]], track_manager.trackwith_id[index[k_]].sig_track_cov_list, track_manager.trackwith_id[index[k_]].sig_track_state_list);
                cout << "x_update=====" << kf.x_ << endl;
                cout << "p_update=====" << kf.p_ << endl;
                track_manager.trackwith_id[index[k_]].sig_track_state_list = kf.x_;
                track_manager.trackwith_id[index[k_]].sig_track_cov_list = kf.p_;

                // out_file << kf.x_[0] << "\n";

            }



           
            cout << "N=============" << N << endl;
            cout << "M=============" << M << endl;
           
            /*  把剩下的测量值添加进轨迹管理列表 */
           
                for (size_t z_ = N+1; z_ <= N + unsign_match_meas.size(); z_++)
                {
                    track_manager.Addtrack_list(z_, unsign_match_meas[z_ - N-1], p);
                }

            
           /*  把剩下的添加置信度 */

              
                for (size_t ui = 0; ui < unsign_match_track_id.size(); ui++){
                
                    track_manager.trackwith_id[unsign_match_track_id[ui]].slot + 1;


                }

                for (size_t sl = 0; sl < N; sl++)
                {
                    if (track_manager.trackwith_id[sl].slot > 1)
                    {
                        /*删除该轨迹*/

                        track_manager.Deletrack_list(sl);
                    }
                   
                }


        }
      
     

        for (size_t k = 0; k < track_manager.trackwith_id.size(); k++)
        {
            cout << "轨迹id===========" << track_manager.trackwith_id[k].id << endl;
        }



     
      

        count_t ++;

        k = k + count - 1;

        j = k-1 ;

    }
  
  


        if (in_file.is_open()) {
            in_file.close();
        }
        return 0;


}

