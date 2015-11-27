
#include "Header.h"
#include "FeatureMatching.h"
#include "WebCamera.h"
#include "SfM.h"

int main()
{
	//�������
		printf("0�F�����_�}�b�`���O\n");
		printf("1 : �J�����L�����u���[�V����\n");
		printf("2 : �J�����E�v���W�F�N�^�̃L�����u���[�V�������ʓǂݍ���\n");
		printf("3: �J�����E�v���W�F�N�^�̑��Έʒu����\n");
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
				SfM sfm("./Image/capture/cap32.jpg", "./Image/capture/cap33.jpg", mainCamera, mainProjector);
				//�@�����_�}�b�`���O�őΉ��_�擾
				sfm.featureMatching("ORB", "ORB", "BruteForce-L1", true);
				sfm.saveResult("./Image/result/result_10.jpg");
				//�A��{�s��̎Z�o
				cv::Mat E = sfm.findEssientialMat();
				std::cout << "\nEssentiamMat:\n" << E << std::endl;

				cv::Mat R = cv::Mat::eye(3,3,CV_64F);
				cv::Mat t = cv::Mat::zeros(3,1,CV_64F);

				//�BR,t�̎Z�o
				sfm.recoverPose(E, R, t);
				//�B��{�s��̕���
				//sfm.findProCamPose(E, R, t);
				std::cout << "\nR:\n" << R << std::endl;
				std::cout << "t:\n" << t << std::endl;

			}
			break;
		default:
			exit(0);
			break;
		}
	}

	return 0;
}