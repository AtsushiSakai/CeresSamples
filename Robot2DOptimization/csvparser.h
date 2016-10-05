/**
 * @file csvparser.h
 *
 * @brief csv file parser
 *
 * @author Atsushi Sakai
*/

#ifndef CSV_PARSER_H
#define CSV_PARSER_H

#include<fstream>
#include<sstream>
#include<iostream>
#include<string>
#include<vector>

using namespace std;

class CSVParser{
  public:
    CSVParser(const string &filename, bool withheader=true){

      ncol_=nrow_=0.0;

      cout<<"file name is "<<filename<<endl;

      ifstream ifs(filename);
      if(!ifs){
        cout<<"Error: Cannot read file";
        return;
      }

      ReadContents(ifs, withheader);
    }


    //Data strage
    vector<vector<double> > data_;
    uint32_t ncol_;
    uint32_t nrow_;

  private:

    void ReadContents(ifstream &ifs, bool withheader){
      //Read each line of csv file 
      string str;
      if(withheader) getline(ifs,str); //remove header


      while(getline(ifs,str)){
        string token;
        istringstream stream(str);

        vector<double> line;

        while(getline(stream,token,',')){
          // cout<<token<<",";

          float temp=stof(token); //stof(string str) : stringをfloatに変換
          // cout<<temp<<",";
          line.push_back(temp);
        }

        nrow_=line.size();

        data_.push_back(line);
      }

      ncol_=data_.size();

      cout<<"ncol:"<<ncol_<<",nrow:"<<nrow_<<" is read"<<endl;
    }

};


#endif  //CSV_PARSER_H

