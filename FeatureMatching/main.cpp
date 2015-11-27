
#include "Header.h"
#include "FeatureMatching.h"
#include "WebCamera.h"
#include "SfM.h"

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
	//操作説明
		printf("0：特徴点マッチング\n");
		printf("1 : カメラキャリブレーション\n");
		printf("2 : カメラ・プロジェクタのキャリブレーション結果読み込み\n");
		printf("3: カメラ・プロジェクタの相対位置推定\n");
		printf("c : 撮影\n"); 

	//カメラ
	WebCamera mainCamera(640, 480, "webCamera0");
	//プロジェクタ
	WebCamera mainProjector(640, 480, "projector0");

	int frame = 0;

	// キー入力受付用の無限ループ
	while(true){
		printf("====================\n");
		printf("数字を入力してください....\n");
		int command;

		//カメラメインループ
		while(true)
		{
			// 何かのキーが入力されたらループを抜ける
			command = cv::waitKey(33);
			if ( command > 0 ){
				//cキーで撮影
				if(command == 'c')
					mainCamera.capture();
				//m1キーで3sに1回100枚連続撮影
				else if(command == 'm')
				{
					while(mainCamera.capture_num < 100)
					{
						Sleep(3000);
						mainCamera.idle();
						mainCamera.capture();
					}
				}
				else break;
			}
			mainCamera.idle();
		}

		// 条件分岐
		switch (command){

		case '0':
			{
				FeatureMatching featureMatching("./Image/movie/cap1.jpg", "./Image/movie/cap2.jpg", "SIFT", "SIFT", "BruteForce-L1", true);
				featureMatching.apply();
				featureMatching.saveResult("./Image/result/result_10.jpg");
				break;
			}
		case '1':
			{
				mainCamera.initCalibration(10, 7, 24.0);
				mainCamera.cameraCalibration();
			}
			break;
		case '2' :
			{
				mainCamera.loadCalibParam("inCame.xml");
				printf("カメラキャリブレーションデータ読み込み\n");
				mainProjector.loadCalibParam("inCame.xml");
				printf("プロジェクタキャリブレーションデータ読み込み\n");
			}
			break;
		case '3':
			{
				//SfM
				SfM sfm("./Image/capture/cap32.jpg", "./Image/capture/cap33.jpg", mainCamera, mainProjector);
				//�@特徴点マッチングで対応点取得
				sfm.featureMatching("ORB", "ORB", "BruteForce-L1", true);
				sfm.saveResult("./Image/result/result_10.jpg");
				//�A基本行列の算出
				cv::Mat E = sfm.findEssientialMat();
				std::cout << "\nEssentiamMat:\n" << E << std::endl;

				cv::Mat R = cv::Mat::eye(3,3,CV_64F);
				cv::Mat t = cv::Mat::zeros(3,1,CV_64F);

				//�BR,tの算出
				sfm.recoverPose(E, R, t);
				//�B基本行列の分解
				//sfm.findProCamPose(E, R, t);
				std::cout << "\nR:\n" << R << std::endl;
				std::cout << "t:\n" << t << std::endl;

				// 3Dビューア
				pcl::visualization::PCLVisualizer viewer("3D Viewer");
				viewer.setBackgroundColor(0, 0, 0);
				viewer.addCoordinateSystem(2.0);
				viewer.initCameraParameters();
				Eigen::Affine3f view;
				Eigen::Matrix4f _t;
				_t << R.at<float>(0,0) , R.at<float>(0,1) , R.at<float>(0,2) , t.at<float>(0,0),
						  R.at<float>(1,0) , R.at<float>(1,1) , R.at<float>(1,2) ,  t.at<float>(1,0),
						  R.at<float>(2,0) , R.at<float>(2,1) , R.at<float>(2,2) ,  t.at<float>(2,0);
				view = _t;
				viewer.addCoordinateSystem(1.0, view);


			}
			break;
		default:
			exit(0);
			break;
		}
	}

	return 0;
}