#include <stdlib.h>
#include <stdint.h>
#include "positioning.h"


float my_sqrt( float number )
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   
    //y  = y * ( threehalfs - ( x2 * y * y ) );  

    y = 1 / y;
	return y;
}

float my_abs(float number){
    int temp = 0;
    float y, num;
    num = number;
    temp = *(int *) &num;
    temp &= (0xffffffff >> 1);
    y = *(float *) &temp;
    return y;
}




float determinant_2(float mat[2][2]){
    float ans = mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0];
    return ans;
}

float determinant_3(float mat[3][3]){
    float ans = mat[0][0] * (mat[1][1]*mat[2][2] - mat[2][1]*mat[1][2]) - mat[0][1] * (mat[1][0]*mat[2][2] - mat[2][0]*mat[1][2]) + mat[0][2] * (mat[1][0]*mat[2][1] - mat[2][0]*mat[1][1]); 
    return ans;
}

float determinant_4(float mat[4][4]){
    float mat_3[3][3] = {0};
    float ans = 0;
    int mat_3_index = 0;
    int sign = 1;
    for(int index = 0; index < 4; index++){
        for(int i = 1; i < 4; i++){
            for(int j = 0; j < 4; j++){
                if(j == index){
                    continue;
                }
                mat_3[i-1][mat_3_index] = mat[i][j];
                mat_3_index += 1;
            }
            mat_3_index = 0;
        }
        ans += mat[0][index]*sign*determinant_3(mat_3);
        sign *= -1;
    }
    return ans;
    
}

void inv_3_matrix(float mat[3][3], float inv_mat[3][3]){
    float coef = 1/determinant_3(mat);
    float adj[2][2] = {0};
    int adj_row = 0, adj_col = 0;
    int sign = 1;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int row = 0; row < 3; row++){
                adj_col = 0;
                if(row == i){
                    continue;
                }
                for(int col = 0; col < 3; col++){
                    if(col == j){
                        continue;
                    }
                    adj[adj_row][adj_col] = mat[row][col];
                    adj_col += 1;
                }
                adj_row += 1;
            }
            if((i+j) % 2 == 0){
                sign = 1;
            }
            else{
                sign = -1;
            }
            inv_mat[i][j] = coef*sign*determinant_2(adj);
            
            adj_row = 0;
        }
    }
}


void inv_4_matrix(float mat[4][4], float inv_mat[4][4]){
    float coef = 1/determinant_4(mat);
    float adj[3][3] = {0};
    int adj_row = 0, adj_col = 0;
    int sign = 1;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int row = 0; row < 4; row++){
                adj_col = 0;
                if(row == i){
                    continue;
                }
                for(int col = 0; col < 4; col++){
                    if(col == j){
                        continue;
                    }
                    adj[adj_row][adj_col] = mat[row][col];
                    adj_col += 1;
                }
                adj_row += 1;
            }
            if((i+j) % 2 == 0){
                sign = 1;
            }
            else{
                sign = -1;
            }
            inv_mat[i][j] = coef*sign*determinant_3(adj);
            
            adj_row = 0;
        }
    }
}


//float distance_list[8] = {a0, a1, a2 ......, a7} 若沒收到距離的anchor填入-1
//float position_result[3] 傳入一個陣列儲存定位結果

void get_position(float *distance_list, float *position_result){
    
    float anchorselected[MAX_NUM_ANCHOR][3] = {0};
    float distanceselected[MAX_NUM_ANCHOR] = {0};
    
    uint8_t num_anchorUse = 0;   
    for(uint8_t a = 0; a < 8; a++){
        if(distance_list[a] != -1 && distance_list[a] > 0 && distance_list[a] < 30){  //anchor只取 0 < d < 30 
            anchorselected[num_anchorUse][0] = anchor[a][0];
            anchorselected[num_anchorUse][1] = anchor[a][1];
            anchorselected[num_anchorUse][2] = anchor[a][2];
            distanceselected[num_anchorUse] = distance_list[a];
            num_anchorUse++;
        }
    }

    uint8_t min_num_anchorUse = 4; //至少要使用5個anchor定位
    if(num_anchorUse < min_num_anchorUse){
        return;
    }

    //定位流程

    float Ga[MAX_NUM_ANCHOR][4] = {0};
    float h[MAX_NUM_ANCHOR] = {0};  
    for(int i = 0; i < num_anchorUse; i++){
        Ga[i][0] = -2*anchorselected[i][0];
        Ga[i][1] = -2*anchorselected[i][1];
        Ga[i][2] = -2*anchorselected[i][2];
        Ga[i][3] = 1;
        h[i] = distanceselected[i]*distanceselected[i] - (anchorselected[i][0]*anchorselected[i][0] + anchorselected[i][1]*anchorselected[i][1] + anchorselected[i][2]*anchorselected[i][2]);
    }


    //float Q[MAX_NUM_ANCHOR] = {1, 1, 1, 1, 1, 1, 1, 1}   Q sets to be the same with each one => Q do not effect weight 
    float Psi[MAX_NUM_ANCHOR] = {0};
    float inv_Psi[MAX_NUM_ANCHOR] = {0};
    for(int i = 0; i < num_anchorUse; i++){
        Psi[i] = 4*distanceselected[i]*distanceselected[i];
        inv_Psi[i] = 1/Psi[i];
    }

    float GaT_invPsi[4][MAX_NUM_ANCHOR] = {0};
    for(int i = 0; i < num_anchorUse; i++){
        for(int j = 0; j < 4; j++){
            GaT_invPsi[j][i] = Ga[i][j]*inv_Psi[i];
        }
    }

    float GaT_invPsi_Ga[4][4] = {0};
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < num_anchorUse; k++){
                GaT_invPsi_Ga[i][j] += GaT_invPsi[i][k]*Ga[k][j]; 
            }
        }
    }

    float inv_GaT_invPsi_Ga[4][4] = {0};
    inv_4_matrix(GaT_invPsi_Ga, inv_GaT_invPsi_Ga);
    
    float GaT_Psi_h[4] = {0};
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < num_anchorUse; j++){
            GaT_Psi_h[i] += GaT_invPsi[i][j]*h[j];
        }
    }

    float za[4] = {0};
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            za[i] += inv_GaT_invPsi_Ga[i][j]*GaT_Psi_h[j];
        }
    }
    
    float hp[4] = {za[0]*za[0], za[1]*za[1], za[2]*za[2], za[3]};
    float Gap[4][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
        {1, 1, 1}
    };
    float bp[4] = {za[0], za[1], za[2], 0.5};
    float Psip[4][4] = {0};
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            Psip[i][j] = 4*inv_GaT_invPsi_Ga[i][j]*bp[i]*bp[j];
        }
    }
    float zap[3] = {0};
    float invPsip[4][4] = {0};
    float GapT_invPsip[3][4] = {0};
    float GapT_invPsip_Gap[3][3] = {0};
    float inv_GapT_invPsip_Gap[3][3] = {0};
    
    inv_4_matrix(Psip, invPsip);
    
    

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < 4; k++){
                GapT_invPsip[i][j] += Gap[k][i]*invPsip[k][j];
            }        
        }
    }

    for(int i = 0; i <3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 4; k++){
                GapT_invPsip_Gap[i][j] += GapT_invPsip[i][k]*Gap[k][j];
            }
        }
    }

    inv_3_matrix(GapT_invPsip_Gap, inv_GapT_invPsip_Gap);

    float GaT_invPsip_hp[3] = {0};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 4; j++){
            GaT_invPsip_hp[i] += GapT_invPsip[i][j]*hp[j];
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            zap[i] += inv_GapT_invPsip_Gap[i][j]*GaT_invPsip_hp[j];
        }
        
    }

    float zp[3] = {0};
    for(int i = 0; i < 3; i++){
        zp[i] = my_sqrt(my_abs(zap[i]));
        position_result[i] = zp[i]; 
    }


}


