#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <string.h>
#include <cmath>
using namespace std;

struct KdTree{
	vector<double> data;
	KdTree* parent;
	KdTree* leftChild;
	KdTree* rightChild;
	//默认构造函数
	KdTree(){parent = leftChild = rightChild = NULL;}
	//判断kd树是否为空
	bool isEmpty()
	{
		return data.empty();
	}
	//判断kd树是否只是一个叶子结点
	bool isLeaf()
	{
		return (!data.empty()) &&
			rightChild == NULL && leftChild == NULL;
	}
	//判断是否是树的根结点
	bool isdata()
	{
		return (!isEmpty()) && parent == NULL;
	}
	//判断该子kd树的根结点是否是其父kd树的左结点
	bool isLeft()
	{
		return parent->leftChild->data == data;
	}
	//判断该子kd树的根结点是否是其父kd树的右结点
	bool isRight()
	{
		return parent->rightChild->data == data;
	}
};

template<typename T>
vector<vector<T> > Transpose(vector<vector<T> > Matrix)
{
	unsigned row = Matrix.size();
	unsigned col = Matrix[0].size();
	vector<vector<T> > Trans(col,vector<T>(row,0));
	for (unsigned i = 0; i < col; ++i)
	{
		for (unsigned j = 0; j < row; ++j)
		{
			Trans[i][j] = Matrix[j][i];
		}
	}
	return Trans;
}

template <typename T>
T findMiddleValue(vector<T> vec)
{
	sort(vec.begin(),vec.end());
    int pos = vec.size() / 2;
	return vec[pos];
}


//构建kd树
void buildKdTree(KdTree* tree, vector<vector<double> > data, unsigned depth)
{
	//样本的数量
	unsigned samplesNum = data.size();
	//终止条件
	if (samplesNum == 0)return;
	if (samplesNum == 1)
	{
		tree->data = data[0];
		return;
	}
	//样本的维度
	unsigned k = data[0].size();
	vector<vector<double> > transData = Transpose(data);
	//选择切分属性
	unsigned splitAttribute = depth % k;
	vector<double> splitAttributeValues = transData[splitAttribute];
	//选择切分值
	double splitValue = findMiddleValue(splitAttributeValues);
	//cout << "splitValue" << splitValue  << endl;

	// 根据选定的切分属性和切分值，将数据集分为两个子集
	vector<vector<double> > subset1;
	vector<vector<double> > subset2;
	for (unsigned i = 0; i < samplesNum; ++i)
	{
		if (splitAttributeValues[i] == splitValue && tree->data.empty())
			tree->data = data[i];
		else
		{
			if (splitAttributeValues[i] < splitValue)
				subset1.push_back(data[i]);
			else
				subset2.push_back(data[i]);
		}
	}

	//子集递归调用buildKdTree函数

	tree->leftChild = new KdTree;
	tree->leftChild->parent = tree;
	tree->rightChild = new KdTree;
	tree->rightChild->parent = tree;
	buildKdTree(tree->leftChild, subset1, depth + 1);
	buildKdTree(tree->rightChild, subset2, depth + 1);
}

//计算空间中两个点的距离
double measureDistance(vector<double> point1, vector<double> point2)
{
	if (point1.size() != point2.size())
	{
		cerr << "Dimensions don't match！！" ;
		exit(1);
	}
	else
    {
        double res = 0;
        for (vector<double>::size_type i = 0; i < point1.size(); ++i)
        {
            res += pow((point1[i] - point2[i]), 2);
        }
        return sqrt(res);
    }
}
//在kd树tree中搜索目标点goal的最近邻
//输入：目标点；已构造的kd树
//输出：目标点的最近邻
int searchNearestNeighbor(vector<double> goal, KdTree *tree)
{
	/*第一步：在kd树中找出包含目标点的叶子结点：从根结点出发，
	递归的向下访问kd树，若目标点的当前维的坐标小于切分点的
	坐标，则移动到左子结点，否则移动到右子结点，直到子结点为
	叶结点为止,以此叶子结点为“当前最近点”
	*/
	unsigned k = tree->data.size();//计算出数据的维数
	unsigned d = 0;//维度初始化为0，即从第1维开始
	KdTree* currentTree = tree;
	vector<double> currentNearest = currentTree->data;
	while(!currentTree->isLeaf())
	{
		unsigned index = d % k;//计算当前维
		if (currentTree->rightChild->isEmpty() || goal[index] < currentNearest[index])
		{
			currentTree = currentTree->leftChild;
		}
		else
		{
			currentTree = currentTree->rightChild;
		}
		++d;
		if(d == k)break;
	}
	currentNearest = currentTree->data;

	/*第二步：递归地向上回退， 在每个结点进行如下操作：
	(a)如果该结点保存的实例比当前最近点距离目标点更近，则以该例点为“当前最近点”
	(b)当前最近点一定存在于某结点一个子结点对应的区域，检查该子结点的父结点的另
	一子结点对应区域是否有更近的点（即检查另一子结点对应的区域是否与以目标点为球
	心、以目标点与“当前最近点”间的距离为半径的球体相交）；如果相交，可能在另一
	个子结点对应的区域内存在距目标点更近的点，移动到另一个子结点，接着递归进行最
	近邻搜索；如果不相交，向上回退*/

	//当前最近邻与目标点的距离
	double currentDistance = measureDistance(goal, currentNearest);

	//如果当前子kd树的根结点是其父结点的左孩子，则搜索其父结点的右孩子结点所代表
	//的区域，反之亦反
	KdTree* searchDistrict;
	if (currentTree->isLeft())
	{
		if (currentTree->parent->rightChild == NULL)
			searchDistrict = currentTree;
		else
			searchDistrict = currentTree->parent->rightChild;
	}
	else
	{
		searchDistrict = currentTree->parent->leftChild;
	}

	//如果搜索区域对应的子kd树的根结点不是整个kd树的根结点，继续回退搜索
	while (searchDistrict->parent != NULL)
	{
		//搜索区域与目标点的最近距离
		double districtDistance = abs(goal[(d+1)%k] - searchDistrict->parent->data[(d+1)%k]);

		//如果“搜索区域与目标点的最近距离”比“当前最近邻与目标点的距离”短，表明搜索
		//区域内可能存在距离目标点更近的点
		if (districtDistance < currentDistance && searchDistrict->parent != NULL )//
		{
			double parentDistance = measureDistance(goal, searchDistrict->parent->data);

			if (parentDistance < currentDistance)
			{
				currentDistance = parentDistance;
				currentTree = searchDistrict->parent;
				currentNearest = currentTree->data;
			}
			if (!searchDistrict->isEmpty())
			{
				double dataDistance = measureDistance(goal, searchDistrict->data);

				if (dataDistance < currentDistance)
				{
					currentDistance = dataDistance;
					currentTree = searchDistrict;
					currentNearest = currentTree->data;
				}
			}
			if (searchDistrict->leftChild != NULL)
			{
				if (searchDistrict->leftChild->data.empty()==false)
                {
                  double leftDistance = measureDistance(goal, searchDistrict->leftChild->data);

				if (leftDistance < currentDistance)
				{
					currentDistance = leftDistance;
					currentTree = searchDistrict;
					currentNearest = currentTree->data;
				}
                }

			}
			if (searchDistrict->rightChild != NULL)
			{
			    if (searchDistrict->leftChild->data.empty()==false)
                {
                    double rightDistance = measureDistance(goal, searchDistrict->rightChild->data);

                    if (rightDistance < currentDistance)
                    {
                        currentDistance = rightDistance;
                        currentTree = searchDistrict;
                        currentNearest = currentTree->data;
                    }
                }

			}
		}//end if

		if (searchDistrict->parent->parent != NULL)
		{
			searchDistrict = searchDistrict->parent->isLeft()?
							searchDistrict->parent->parent->rightChild:
							searchDistrict->parent->parent->leftChild;
		}
		else
		{
			searchDistrict = searchDistrict->parent;
		}
		++d;
	}//end while
	return currentDistance;
}


void file(string infilename,string outfilename1,string outfilename2)//分离数据集
{

    ifstream infile(infilename.c_str());
    ofstream outfile1(outfilename1.c_str());
    ofstream outfile2(outfilename2.c_str());
    if(!infile)
        cout<<"error"<<endl;
    int count = 0;
    string str1;      //读取的每行数据
    cout<<"分离源文件中,请稍侯"<<endl;
    while(getline(infile,str1))
    {
        vector<double> line;
        int pos,k = 0;
        for(int i = 0;i< str1.size();i++)
        {
           if(str1.at(i)==',')
           {
               k++;
                if(k > 20 && k < 31)
                {
                    pos = i;
                   while(str1.at(++pos) != ',');

                   stringstream ss(str1.substr(i+1,pos-i-1));
                   double temp;
                   ss >> temp;
                   line.push_back(temp);
                }
                if(k==41)
                {
                   if(str1.substr(i+1,6).compare("normal") == 0)
                   {
                       outfile1<<str1<<"\n";
                   }
                   else
                   {
                       outfile2<<str1<<"\n";
                   }
                }
            }
        }
        cout<<"正在分离第"<<cout<<"行数据"<<endl;
        count++;
        if (count == 100000)
            break;
    }
    infile.close();
    outfile1.close();
    outfile2.close();
    return ;
}

vector<vector<double> > datain;//训练集
vector<vector<double> > test;//测试集
int tag[10000]; //标记数组
int linenum = 0;

void file2(string outfilename1,string outfilename2)
{

    ifstream infile(outfilename1.c_str());
    ifstream infile1(outfilename2.c_str());
    if(!infile)
        cout<<"error"<<endl;
    int count = 0;
    int count1 = 0;
    string str1;      //读取的每行数据
    while(getline(infile,str1))
    {
        vector<double> line;
        int pos,k = 0;
        for(int i = 0;i< str1.size();i++)
        {
           if(str1.at(i)==',')
           {
               k++;
                if(k > 20 && k < 31)
                {
                    pos = i;
                   while(str1.at(++pos) != ',');

                   stringstream ss(str1.substr(i+1,pos-i-1));
                   double temp;
                   ss >> temp;
                   line.push_back(temp);
                }

            }
        }
        datain.push_back(line);
        count++;
        if (count == 10000)
            break;
    }

    while(getline(infile1,str1))
    {
        vector<double> line;
        int pos,k = 0;
        for(int i = 0;i< str1.size();i++)
        {
           if(str1.at(i)==',')
           {
               k++;
                if(k > 20 && k < 31)
                {
                    pos = i;
                   while(str1.at(++pos) != ',');

                   stringstream ss(str1.substr(i+1,pos-i-1));
                   double temp;
                   ss >> temp;
                   line.push_back(temp);
                }

            }
           }
            if(str1.find("normal")!=string::npos)
            {
                tag[linenum] = 1;
                linenum++;
            }
           else
            {
            tag[linenum]= -1;
            linenum++;
            }

        test.push_back(line);
        count1++;
        if (count1 == 10000)
            break;
    }
    infile.close();
    infile1.close();
    return ;
}

void PostOrderTravse(KdTree* kdTree)
{
    if (kdTree != NULL)
    {
        PostOrderTravse(kdTree->leftChild);
        PostOrderTravse(kdTree->rightChild);
        delete kdTree;
    }
}

int main()
{
    string infilename  =  "kddcup.data_10_percent";
    string outfilename1 =  "kddcup.123";
    string outfilename2 =  "kddcup.456";
    string outfilename3 =  "kddcup.789";
    //file(infilename,outfilename1,outfilename2);  //分离数据

    file2(outfilename1,outfilename2); //读取数据
    //file2(outfilename1,outfilename3); //读取数据
    cout<<"读取了"<<datain.size()<<"个数据作为训练"<<endl;
    cout<<"读取了"<<test.size()<<"个数据作为攻击"<<endl;
    KdTree* kdTree = new KdTree;
    buildKdTree(kdTree, datain, 0);

    int attack = 0;
    int normal = 0;
    int missreport = 0;
    double boundary = 0.01;
    double nearestDistance = 0;

    for (unsigned i = 0; i <test.size() ; ++i)
    {
       if(i%100 == 0)cout<<"第....................................."<<i<<"次验证"<<endl;
       nearestDistance = searchNearestNeighbor(test[i], kdTree);
       if(nearestDistance < boundary)
        {
            //cout << "不存在攻击"<<"最近距离为："<<nearestDistance <<endl;
            if(tag[i]==1)
            {
              missreport++;
            }

        }
        else
        {
            //cout << "存在攻击"<<"最近距离为："<<nearestDistance <<endl;
            if(tag[i]==1)
            {
               normal++;

            }
            else{
                 attack++;
            }
        }
    }
    cout<<endl<<"正常的次数为："<< normal;
    cout<<endl<<"攻击的次数为："<< attack;
    double DR = double(attack)/(test.size()/2);
    double FPR = double(missreport)/(test.size()/2);

    cout<<endl<<"测试"<<test.size()<<"个样本后DR为："<<DR<<endl;
    cout<<endl<<"测试"<<test.size()<<"个样本后FPR为："<<FPR<<endl;
    cout<<"此时阈值为："<<boundary;
    PostOrderTravse(kdTree);

    return 0;
}
