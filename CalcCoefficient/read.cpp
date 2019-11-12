#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>

int main(int argc, char *argv[]){
    std::ifstream ifs;
    std::string buf, str;
    std::string data_name = "deg.csv";
    int count = 0;

    ifs.open(data_name,std::ios::in);/* csv file open */
    while(getline(ifs,str))
    {
        count++;
    }
    ifs.close();
    ifs.open(data_name,std::ios::in);
    std::vector<double>data(15,0);
    for(int i=0;i<data.size();i++) data[i]=0.f;

/*
    if(ifs && std::getline(ifs, buf)){
        std::istringstream stream(buf);
        for(int j=0; j<15; j++){
            std::string num;
            std::getline(stream, num, ' ');
            data[j] = std::atof(num.c_str());
            std::cout << data[j] << std::endl;
        }
        std::cout << std::endl;
    }
*/

    for(int i=0;i<count;i++){
        if(ifs && std::getline(ifs, buf)){
            std::istringstream stream(buf);
            for(int j=0; j<15; j++){
                std::string num;
                std::getline(stream, num, ' ');
                data[j] = std::atof(num.c_str());
                std::cout << data[j] << std::endl;
            }
            std::cout << std::endl;
        }
    }

    return 0;
}
