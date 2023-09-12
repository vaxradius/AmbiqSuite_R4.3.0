

static int32_t first_integrator = 0;
static int32_t first_result = 0;
static int32_t second_integrator = 0;
//static int32_t second_result = 0;

//Derrived from  http://www.danbullard.com/dan/A_Sigma_Delta_ADC_Simulator_in_C.pdf
void two_level_sigma_delta(uint32_t *out, int16_t in, uint16_t len)
{
	uint16_t i,j = 0;
	int32_t y = in;
	y >>= 1;

	for(i = 0; i < len ; i++)
	{
		*(out+i) = 0;
		for(j=0;j<32;j++)
		{
			second_integrator += (y -(first_result));
			first_integrator += ((second_integrator) -(first_result));

			if (first_integrator > 0) 
			{			
				first_result = 32768;
				*(out+i) |= (1 << j);
			}
			else
			{
				first_result = -32768;
			}
		}
	}
}

//Derrived from https://books.google.com.tw/books?id=Gum3CgAAQBAJ&pg=PA120&dq=Delta-sigma+Modulators+0.1158&hl=zh-TW&sa=X&ved=2ahUKEwiJ6Key4q3sAhVHyosBHQdADwQQ6AEwAHoECAAQAg#v=onepage&q&f=false
void three_level_sigma_delta(uint16_t *out, int16_t in, uint16_t len)
{
		uint16_t i,j = 0;

        static float qe0 = 0;
        static float qe1 = 0;
        static float qe2 = 0;


        static float sum_value0 = 0;
        static float sum_value1 = 0;
        static float sum_value2 = 0;
        static float last_dsm_signal = 0;

        static float k1 = 0.1158;
        static float k2 = 0.2776;
        static float k3 = 1.0267;
        static float g = 0.00153846;
		float u = ((float) in)/(65536.0);

        for(i = 0; i < len ; i++)
		{
			*(out+i) = 0;
			for(j=0;j<16;j++)
			{
                qe0 = k1 * u - k1*last_dsm_signal; 
                sum_value0 = sum_value0 + qe0;


                qe1 = sum_value0 - k2*last_dsm_signal - g*sum_value2;

                sum_value1 = sum_value1 + qe1;

                qe2 = sum_value1 - k3*last_dsm_signal;

                sum_value2 = sum_value2 + qe2;


                if(sum_value2 > 0)
				{
					last_dsm_signal = 1;
					*(out+i) |= (1 << j);                         
                } 
				else 
				{
                    last_dsm_signal = -1;
                }

        	}
        }
}

void PCM_to_PDM(uint32_t *pdm, int32_t *pcm)
{
	uint32_t j = 0;

	for(j=0; j<BUF_SIZE; j++)
	{
		//Am_Sigma_Delta(pdm+(j*(OSR/16)), (*(pcm+j))*2, (OSR/16));
		//PDM_2_PWM(pdm+(j*(OSR/16)));
		//two_level_sigma_delta(pdm+(j*(OSR/16)), (*(pcm+j)), (OSR/16));
		three_level_sigma_delta(pdm+(j*(OSR/16)), (*(pcm+j)), (OSR/16));
	}
}
