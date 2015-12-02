
#include "Header.h"
#include "FeatureMatching.h"
#include "WebCamera.h"
#include "SfM.h"

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


//**Loading datas**//
cv::Mat K1 = cv::Mat::eye(3,3,CV_64F);
cv::Mat K2 = cv::Mat::eye(3,3,CV_64F);
cv::Mat R2 = cv::Mat::eye(3,3,CV_64F);
cv::Mat t2;

std::vector<cv::Point3d> worldPoints; //�Ή��_��3�������W
std::vector<cv::Point2d> imagePoints1; //�J����1�摜�ւ̎ˉe�_
std::vector<cv::Point2d> imagePoints2; //�J����2�摜�ւ̎ˉe�_

void loadFile(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	cv::FileNode node(fs.fs, NULL);

	read(node["worldPoints"], worldPoints);
	read(node["imagePoints1"], imagePoints1);
	read(node["imagePoints2"], imagePoints2);

	read(node["K1"], K1);
	read(node["K2"], K2);
	read(node["R2"], R2);
	read(node["t2"], t2);

	std::cout << "file loaded." << std::endl;
}

int main()
{
	//�������
		printf("0�F�����_�}�b�`���O\n");
		printf("1 : �J�����L�����u���[�V����\n");
		printf("2 : �J�����E�v���W�F�N�^�̃L�����u���[�V�������ʓǂݍ���\n");
		printf("3: �J�����E�v���W�F�N�^�̑��Έʒu����\n");
		printf("4: GroundTruth\n");
		printf("c : �B�e\n"); 

	//�J����
	WebCamera mainCamera(640, 480, "webCamera0");
	//�v���W�F�N�^
	WebCamera mainProjector(640, 480, "projector0");

	int frame = 0;

	// �L�[���͎�t�p�̖������[�v
	while(true){
		printf("====================\n");
		printf("��������͂��Ă�������....\n");
		int command;

		//�J�������C�����[�v
		while(true)
		{
			// �����̃L�[�����͂��ꂽ�烋�[�v�𔲂���
			command = cv::waitKey(33);
			if ( command > 0 ){
				//c�L�[�ŎB�e
				if(command == 'c')
					mainCamera.capture();
				//m1�L�[��3s��1��100���A���B�e
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

		// ��������
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
				printf("�J�����L�����u���[�V�����f�[�^�ǂݍ���\n");
				mainProjector.loadCalibParam("inCame.xml");
				printf("�v���W�F�N�^�L�����u���[�V�����f�[�^�ǂݍ���\n");
			}
			break;
		case '3':
			{
				//SfM
				SfM sfm("./Image/capture/cap38.jpg", "./Image/capture/cap40.jpg", mainCamera, mainProjector);
				//�@�����_�}�b�`���O�őΉ��_�擾
				sfm.featureMatching("ORB", "ORB", "BruteForce-L1", true);
				sfm.saveResult("./Image/result/result_10.jpg");
				//�A��{�s��̎Z�o
				cv::Mat E1 = sfm.findEssentialMat(); //cv::calibrationMatrixValues
				cv::Mat E2 = sfm.findEssentialMat2();//�����s��̋t�s����|����
				//std::cout << "\nEssentiamMat:\n" << E << std::endl;

				//cv::Mat R = cv::Mat::eye(3,3,CV_64F);
				//cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
				cv::Matx33d R1, R2;
				cv::Matx31d t1, t2;

				//�BR,t�̎Z�o
				sfm.recoverPose(E1, R1, t1);
				sfm.recoverPose(E2, R2, t2);
				//�B��{�s��̕���
				//sfm.findProCamPose(E, R, t);
				std::cout << "\nR1:\n" << R1 << std::endl;
				std::cout << "t1:\n" << t1 << std::endl;
				std::cout << "\nR2:\n" << R2 << std::endl;
				std::cout << "t2:\n" << t2 << std::endl;

				// 3D�r���[�A
				pcl::visualization::PCLVisualizer viewer("3D Viewer");
				viewer.setBackgroundColor(0, 0, 0);
				viewer.addCoordinateSystem(2.0);
				viewer.initCameraParameters();
				Eigen::Affine3f view1, view2;
				Eigen::Matrix4f _t1, _t2;
				//E1�̌���
				_t1 << R1(0,0) , R1(0,1) , R1(0,2) , t1(0,0), 
						  R1(1,0) , R1(1,1) , R1(1,2) , t1(1,0), 
						  R1(2,0) , R1(2,1) , R1(2,2) , t1(2,0), 
						  0.0f, 0.0f ,0.0f, 1.0f;
				std::cout << "_t1:\n"<< _t1 <<std::endl;
				view1 = _t1;
				viewer.addCoordinateSystem(1.0, view1);
				//E2�̌���
				_t2 << R2(0,0) , R2(0,1) , R2(0,2) , t2(0,0), 
						  R2(1,0) , R2(1,1) , R2(1,2) , t2(1,0), 
						  R2(2,0) , R2(2,1) , R2(2,2) , t2(2,0), 
						  0.0f, 0.0f ,0.0f, 1.0f;
				std::cout << "_t2:\n"<< _t2 <<std::endl;
				view2 = _t2;
				viewer.addCoordinateSystem(0.5, view2);


			}
			break;
			case '4':
			{
				//data loading
				loadFile("../groundtruth_1222032.xml");

				//cv::FileStorage fs("../groundtruth_1221634.xml", cv::FileStorage::READ);
				//cv::FileNode node(fs.fs, NULL);

				//read(node["worldPoints"], worldPoints);
				//read(node["imagePoints1"], imagePoints1);
				//read(node["imagePoints2"], imagePoints2);

				//read(node["K1"], K1);
				//read(node["K2"], K2);
				//read(node["R2"], R2);
				//read(node["t2"], t2);

				//std::cout << "file loaded." << std::endl;

				std::vector<cv::Point2d> p1;//camera
				std::vector<cv::Point2d> p2;//projector

				//�Ή��t���������_�̎��o���Əœ_����1.0�̂Ƃ��̍��W�ɕϊ�
				for (size_t i = 0; i < imagePoints1.size(); ++i)
				{
				  cv::Mat ip(3, 1, CV_64FC1);
				  cv::Point2d p;
				  //�J�����̓_
				  ip.at<double>(0) = imagePoints1[i].x;
				  ip.at<double>(1) = imagePoints1[i].y;
				  ip.at<double>(2) = 1.0;
				  ip = K1.inv()*ip;
				  p.x = ip.at<double>(0);
				  p.y = ip.at<double>(1);
				  p1.push_back( p );
				  //�v���W�F�N�^�̓_
				  ip.at<double>(0) = imagePoints2[i].x;
				  ip.at<double>(1) = imagePoints2[i].y;
				  ip.at<double>(2) = 1.0;
				  ip = K2.inv()*ip;
				  p.x = ip.at<double>(0);
				  p.y = ip.at<double>(1);
				  p2.push_back( p );
				}

				cv::Mat E = cv::findEssentialMat(p1, p2);
				std::cout << "Essential Matrix\n" << E << std::endl;

				cv::Mat R1 = cv::Mat::eye(3,3,CV_64F);
				cv::Mat t1 = cv::Mat::zeros(3,1,CV_64F);
				cv::Matx33d R2_calc;
				cv::Matx31d t2_calc;

				//�BR,t�̎Z�o
				cv::recoverPose(E, p1, p2, R2_calc, t2_calc);

				std::cout << "\nR1:\n" << R1 << std::endl;
				std::cout << "t1:\n" << t1 << std::endl;
				std::cout << "\nground truth\n R2:\n" << R2 << std::endl;
				std::cout << "t2:\n" << t2 << std::endl;
				std::cout << "\ncalc result\n R2:\n" << R2_calc << std::endl;
				std::cout << "t2:\n" << t2_calc << std::endl;


			}
			break;
			default:
			exit(0);
			break;
		}
	}

	return 0;
}
