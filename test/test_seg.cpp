#include <segment_common.h>
#include <cluster_extraction.h>
#include <dirent.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
using namespace std;
typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud_;

//vector<string> getFiles(string cate_dir);


int main(int argc, char const *argv[]) {
//  string filePath = "/home/yuechen/intern/seg/pcd";
//  vector<string> files;

  ////获取该路径下的所有文件
//  files = getFiles(filePath);


  Segment_pallet *sp = new Segment_pallet();
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
//  for (int i = 0; i < files.size(); i++) {
//    cout << files[i] << endl;
    sp->initialize("/home/yuechen/pcd_file/pallet.pcd", "/home/yuechen/intern/simulate/data/2019-04-01-10-06-58.pcd");
    cout << "segmentation starts:" << endl;
    sp->cluster_extraction();
    cout << "end of the segmentation" << endl;

//  }
  return 0;
}


/*vector<string> getFiles(string cate_dir)
{
	vector<string> files;//存放文件名

	DIR *dir;
	struct dirent *ptr;
	char base[1000];

	if ((dir=opendir(cate_dir.c_str())) == NULL)
        {
		perror("Open dir error...");
                exit(1);
        }

	while ((ptr=readdir(dir)) != NULL)
	{
		if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
		        continue;
		else if(ptr->d_type == 8)    ///file
			//printf("d_name:%s/%s\n",basePath,ptr->d_name);
			files.push_back(cate_dir + "/" + ptr->d_name);
		else if(ptr->d_type == 10)    ///link file
			//printf("d_name:%s/%s\n",basePath,ptr->d_name);
			continue;
		else if(ptr->d_type == 4)    ///dir
		{
			files.push_back(cate_dir + "/" + ptr->d_name);

		        memset(base,'\0',sizeof(base));
		        strcpy(base,basePath);
		        strcat(base,"/");
		        strcat(base,ptr->d_nSame);
		        readFileList(base);

		}
	}
	closedir(dir);


	//排序，按从小到大排序
//	sort(files.begin(), files.end());
	return files;
}*/
