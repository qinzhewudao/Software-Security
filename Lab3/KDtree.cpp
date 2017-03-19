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
	//Ĭ�Ϲ��캯��
	KdTree(){parent = leftChild = rightChild = NULL;}
	//�ж�kd���Ƿ�Ϊ��
	bool isEmpty()
	{
		return data.empty();
	}
	//�ж�kd���Ƿ�ֻ��һ��Ҷ�ӽ��
	bool isLeaf()
	{
		return (!data.empty()) &&
			rightChild == NULL && leftChild == NULL;
	}
	//�ж��Ƿ������ĸ����
	bool isdata()
	{
		return (!isEmpty()) && parent == NULL;
	}
	//�жϸ���kd���ĸ�����Ƿ����丸kd��������
	bool isLeft()
	{
		return parent->leftChild->data == data;
	}
	//�жϸ���kd���ĸ�����Ƿ����丸kd�����ҽ��
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


//����kd��
void buildKdTree(KdTree* tree, vector<vector<double> > data, unsigned depth)
{
	//����������
	unsigned samplesNum = data.size();
	//��ֹ����
	if (samplesNum == 0)return;
	if (samplesNum == 1)
	{
		tree->data = data[0];
		return;
	}
	//������ά��
	unsigned k = data[0].size();
	vector<vector<double> > transData = Transpose(data);
	//ѡ���з�����
	unsigned splitAttribute = depth % k;
	vector<double> splitAttributeValues = transData[splitAttribute];
	//ѡ���з�ֵ
	double splitValue = findMiddleValue(splitAttributeValues);
	//cout << "splitValue" << splitValue  << endl;

	// ����ѡ�����з����Ժ��з�ֵ�������ݼ���Ϊ�����Ӽ�
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

	//�Ӽ��ݹ����buildKdTree����

	tree->leftChild = new KdTree;
	tree->leftChild->parent = tree;
	tree->rightChild = new KdTree;
	tree->rightChild->parent = tree;
	buildKdTree(tree->leftChild, subset1, depth + 1);
	buildKdTree(tree->rightChild, subset2, depth + 1);
}

//����ռ���������ľ���
double measureDistance(vector<double> point1, vector<double> point2)
{
	if (point1.size() != point2.size())
	{
		cerr << "Dimensions don't match����" ;
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
//��kd��tree������Ŀ���goal�������
//���룺Ŀ��㣻�ѹ����kd��
//�����Ŀ���������
int searchNearestNeighbor(vector<double> goal, KdTree *tree)
{
	/*��һ������kd�����ҳ�����Ŀ����Ҷ�ӽ�㣺�Ӹ���������
	�ݹ�����·���kd������Ŀ���ĵ�ǰά������С���зֵ��
	���꣬���ƶ������ӽ�㣬�����ƶ������ӽ�㣬ֱ���ӽ��Ϊ
	Ҷ���Ϊֹ,�Դ�Ҷ�ӽ��Ϊ����ǰ����㡱
	*/
	unsigned k = tree->data.size();//��������ݵ�ά��
	unsigned d = 0;//ά�ȳ�ʼ��Ϊ0�����ӵ�1ά��ʼ
	KdTree* currentTree = tree;
	vector<double> currentNearest = currentTree->data;
	while(!currentTree->isLeaf())
	{
		unsigned index = d % k;//���㵱ǰά
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

	/*�ڶ������ݹ�����ϻ��ˣ� ��ÿ�����������²�����
	(a)����ý�㱣���ʵ���ȵ�ǰ��������Ŀ�����������Ը�����Ϊ����ǰ����㡱
	(b)��ǰ�����һ��������ĳ���һ���ӽ���Ӧ�����򣬼����ӽ��ĸ�������
	һ�ӽ���Ӧ�����Ƿ��и����ĵ㣨�������һ�ӽ���Ӧ�������Ƿ�����Ŀ���Ϊ��
	�ġ���Ŀ����롰��ǰ����㡱��ľ���Ϊ�뾶�������ཻ��������ཻ����������һ
	���ӽ���Ӧ�������ڴ��ھ�Ŀ�������ĵ㣬�ƶ�����һ���ӽ�㣬���ŵݹ������
	����������������ཻ�����ϻ���*/

	//��ǰ�������Ŀ���ľ���
	double currentDistance = measureDistance(goal, currentNearest);

	//�����ǰ��kd���ĸ�������丸�������ӣ��������丸�����Һ��ӽ��������
	//�����򣬷�֮�෴
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

	//������������Ӧ����kd���ĸ���㲻������kd���ĸ���㣬������������
	while (searchDistrict->parent != NULL)
	{
		//����������Ŀ�����������
		double districtDistance = abs(goal[(d+1)%k] - searchDistrict->parent->data[(d+1)%k]);

		//���������������Ŀ����������롱�ȡ���ǰ�������Ŀ���ľ��롱�̣���������
		//�����ڿ��ܴ��ھ���Ŀ�������ĵ�
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


void file(string infilename,string outfilename1,string outfilename2)//�������ݼ�
{

    ifstream infile(infilename.c_str());
    ofstream outfile1(outfilename1.c_str());
    ofstream outfile2(outfilename2.c_str());
    if(!infile)
        cout<<"error"<<endl;
    int count = 0;
    string str1;      //��ȡ��ÿ������
    cout<<"����Դ�ļ���,���Ժ�"<<endl;
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
        cout<<"���ڷ����"<<cout<<"������"<<endl;
        count++;
        if (count == 100000)
            break;
    }
    infile.close();
    outfile1.close();
    outfile2.close();
    return ;
}

vector<vector<double> > datain;//ѵ����
vector<vector<double> > test;//���Լ�
int tag[10000]; //�������
int linenum = 0;

void file2(string outfilename1,string outfilename2)
{

    ifstream infile(outfilename1.c_str());
    ifstream infile1(outfilename2.c_str());
    if(!infile)
        cout<<"error"<<endl;
    int count = 0;
    int count1 = 0;
    string str1;      //��ȡ��ÿ������
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
    //file(infilename,outfilename1,outfilename2);  //��������

    file2(outfilename1,outfilename2); //��ȡ����
    //file2(outfilename1,outfilename3); //��ȡ����
    cout<<"��ȡ��"<<datain.size()<<"��������Ϊѵ��"<<endl;
    cout<<"��ȡ��"<<test.size()<<"��������Ϊ����"<<endl;
    KdTree* kdTree = new KdTree;
    buildKdTree(kdTree, datain, 0);

    int attack = 0;
    int normal = 0;
    int missreport = 0;
    double boundary = 0.01;
    double nearestDistance = 0;

    for (unsigned i = 0; i <test.size() ; ++i)
    {
       if(i%100 == 0)cout<<"��....................................."<<i<<"����֤"<<endl;
       nearestDistance = searchNearestNeighbor(test[i], kdTree);
       if(nearestDistance < boundary)
        {
            //cout << "�����ڹ���"<<"�������Ϊ��"<<nearestDistance <<endl;
            if(tag[i]==1)
            {
              missreport++;
            }

        }
        else
        {
            //cout << "���ڹ���"<<"�������Ϊ��"<<nearestDistance <<endl;
            if(tag[i]==1)
            {
               normal++;

            }
            else{
                 attack++;
            }
        }
    }
    cout<<endl<<"�����Ĵ���Ϊ��"<< normal;
    cout<<endl<<"�����Ĵ���Ϊ��"<< attack;
    double DR = double(attack)/(test.size()/2);
    double FPR = double(missreport)/(test.size()/2);

    cout<<endl<<"����"<<test.size()<<"��������DRΪ��"<<DR<<endl;
    cout<<endl<<"����"<<test.size()<<"��������FPRΪ��"<<FPR<<endl;
    cout<<"��ʱ��ֵΪ��"<<boundary;
    PostOrderTravse(kdTree);

    return 0;
}
