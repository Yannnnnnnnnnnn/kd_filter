#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <malloc.h>
#include "kdtree.h"

#define MAX_LINE 255


int random_index(int start, int end)
{
    
    int dis = end - start;
    return (int)(start + dis * (rand() / (RAND_MAX + 1.0)));
}

int main(int argc,char *argv[]) 
{
    if(argc<3)
    {
        printf("Invalid input parameter\n");
        printf("argv[1] for input points \n");
        printf("argv[2] for output points \n");

        return -1;
    }

    printf("Points Filter\n");

    void* tree = kd_create(3);

    // Load point cloud from file
    FILE *point_cloud_file = NULL;
    point_cloud_file = fopen(argv[1], "r");

    // Get Points Number
    char str[MAX_LINE];
    int points_number = 0;
    fgets(str, MAX_LINE, point_cloud_file); // Skip the first line
    while (fgets(str, MAX_LINE, point_cloud_file) != NULL)
          ++points_number;

    // Go to the file head
    rewind(point_cloud_file);
    fgets(str, MAX_LINE, point_cloud_file); // Skip the first line

    // Malloc memory
    double *data = (double*)malloc(sizeof(double)*3*points_number);

    // Load data
    for(int i=0;i<points_number;i++)
    {
        fscanf(point_cloud_file,"%lf,%lf,%lf",data+i*3+0,data+i*3+1,data+i*3+2);
    }
    fclose(point_cloud_file);

    // Build KD-Tree
    for(int i=0;i<points_number;i++)
    {
        kd_insert3(tree,data[i*3+0], data[i*3+1],data[i*3+2],NULL);
    }

    int N = 10;
    srand((unsigned)time(NULL));
    // Average Distance
    double average_distance = 0;
    int part_point_distance = points_number*0.1;
    for(int i=0;i<part_point_distance;i++)
    {
        int index = random_index(0,part_point_distance);

        double pos_temp[3];

        void* res = kd_nearest_n3(tree, data[index*3 + 0],data[index*3 + 1],data[index*3 + 2], N);
        double distance_average_each = 0;
        while(!kd_res_end(res)) 
        {
            kd_res_item(res, pos_temp);
            //printf("%f %f %f %f\n", pos_temp[0], pos_temp[1], pos_temp[2],kd_res_dist(res));
            distance_average_each += kd_res_dist(res);
            kd_res_next(res);
        }

        distance_average_each /= N;
        average_distance += distance_average_each;

        kd_res_free(res);
    }
    average_distance /= part_point_distance;
    printf("Average Distance: %lf \n",average_distance);

    // Filter Noise
    FILE *point_cloud_filter_file = NULL;
    point_cloud_filter_file = fopen(argv[2], "w+");
    fprintf(point_cloud_filter_file,"#X,Y,Z\n");
    for(int i=0;i<points_number;i++)
    {
        double pos_temp[3];

        void* res = kd_nearest_n3(tree, data[i*3 + 0],data[i*3 + 1],data[i*3 + 2], N);
        double distance_average_each = 0;
        while(!kd_res_end(res)) 
        {
            kd_res_item(res, pos_temp);
            //printf("%f %f %f %f\n", pos_temp[0], pos_temp[1], pos_temp[2],kd_res_dist(res));
            distance_average_each += kd_res_dist(res);
            kd_res_next(res);
        }

        distance_average_each /= N;
        
        if( distance_average_each<(2*average_distance) )
        {
            fprintf(point_cloud_filter_file,"%f,%f,%f\n",data[i*3 + 0],data[i*3 + 1],data[i*3 + 2]);
        } 


        kd_res_free(res);
    }

    fclose(point_cloud_filter_file);
    

	kd_free(tree);
    free(data);


    return 0;
}
