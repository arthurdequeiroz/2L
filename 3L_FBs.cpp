#include "dll.h"
#include <windows.h>
#include <math.h>

#define hpwm 1E-04

extern "C" {
    
    DLLIMPORT void SimulationStep(
        double t, double delt, double *in, double *out,
        int *pnError, char * szErrorMsg,
        void ** ptrUserData, int nThreadIndex, void * pAppPtr)
    {
        // Constantes do Sistema
        static float pi = 3.14159265;
        static float fg = 60.0, wg = 2.0 * pi * fg; // Frequência angular
        static float Vgm = 110 * sqrt(2);           // Amplitude da tensão da rede
        static float Vel_ref = 110 * sqrt(2);       // Tensão de referência da carga

        // Parâmetros de Controle
        static double kpc = 0.5, kic = 20.0; // Controlador PI do barramento
        static double kp_res = 10.0, ki_res = 1000.0; // Controlador PI ressonante

        // Controle Barramento 2L
        double Vc2L_ref = 465;
        double Ea = Vc2L_ref;
        double P_error = 0.0, I_error = 0.0;
        double Ig = 0.0, Ig0 = 3.0;

        // Variáveis do Controle de Corrente e PLL
        static float td = 0.0;
        static double ig_ref = 0.0, ig_error = 0.0, ig_error_ressonante = 0.0;
        static double xa = 0.0, xb = 0.0, ei1 = 0.0, ei1a = 0.0;

        // Estado das Chaves
        static bool qsa = 0, qla = 0;
        
        // Variáveis de Controle PLL
        static double theta = 0.0, wf = 0.0, Pd = 0.0;
        static double X1a = 0.0, X2a = 0.0, X1af = 0.0, X2af = 0.0;
        static double dX1a = 0.0, dX2a = 0.0, kix = 1.15, kpx = 0.15;
        static double A1 = -8.796 * 5.0, A2 = -39.48 * 5.0 * 5.0;
        static double A3 = 1.0, A4 = 0.0, B1 = 1.0, B2 = 0.0;
        static double C1 = 0.0, C2 = 39.48 * 5.0 * 5.0, D1 = 0.0;

        // Parâmetros de Simulação
        static double tsave = 0, hsave = 10E-6, tsave0 = 0;
        static int n = 0;
        
        
        // Variáveis de Entrada
        double vtri = in[0]; // Onda triangular como entrada
        double eg = in[1]; // Tensão da rede
        double ig = in[2]; // Corrente da carga // Corrente Shunt
        double vc2L = in[3]; // Tensão Barramento 2L

        // Calcular a corrente real da rede `ig`
       

        // Implementação da PLL (Controle de Fase)
        if (td <= t) {
            td += hpwm;
            
            // Calculo do sinal de fase e sincronismo
            Pd = eg * cos(theta);
            
            // Filtro passa-baixa para o controle de fase
            X1a = X1af;
            X2a = X2af;
            
            dX1a = A1 * X1a + A2 * X2a + B1 * Pd;
            dX2a = A3 * X1a + A4 * X2a + B2 * Pd;
            
            X1af = X1a + dX1a * hpwm;
            X2af = X2a + dX2a * hpwm;
            
            double Pdf = C1 * X1af + C2 * X2af + D1 * Pd;
            double e_pd = -(0.0 - Pdf);
            
            // Controlador PI para ajuste de frequência
            static double X1t = 0.0, dX1t = 0.0;
            X1t += kix * e_pd * hpwm;
            dX1t = X1t + kpx * e_pd;
            
            wf = 2 * pi * fg + dX1t; // Frequência corrigida
            theta += wf * hpwm; // Fase corrigida
            
            if (theta >= 2.0 * pi) theta -= 2.0 * pi; // Resetar a fase
            if (theta < 0.0) theta += 2.0 * pi;
        }

        // Controle PI do Barramento 2L
        double Vc2L_error = Vc2L_ref - vc2L;
        P_error = kpc * Vc2L_error;
        I_error += hpwm * kic * Vc2L_error;

        // Corrente de referência de amplitude com saturação
        Ig = P_error + I_error + Ig0;
        
        // Saturação aplicada à amplitude de Ig
        if (Ig > 30) Ig = 30;
        if (Ig < -30) Ig = -30;

        // Corrente de referência final sincronizada pela PLL
        //Ig = 10;
        ig_ref = Ig * sin(theta);

        // Controle de Corrente Ressonante
        ig_error = ig_ref - ig; // Comparando com corrente real da rede
        ig_error_ressonante = -ig_error;
        
        ei1a = ei1;
        ei1 = -ig_error_ressonante;
        
        // Controle ressonante baseado em estado
        double F1 = cos(wg * hpwm);
        double F2 = sin(wg * hpwm);
        double F3 = 2.0 * ki_res * sin(wg * hpwm);
        double F4 = -wg * sin(wg * hpwm);
        double F5 = cos(wg * hpwm);
        double F6 = (cos(wg * hpwm) - 1.0) * 2.0 * ki_res;
        
        double xa_prev = xa;
        double xb_prev = xb;
        xa = F1 * xa_prev + F2 * xb_prev + F3 * ei1a;
        xb = F4 * xa_prev + F5 * xb_prev + F6 * ei1a;
        
        // Limitar o controle de saída para evitar saturação
        if (xa > 1*Ea) xa = 1*Ea;//mudei os limites do saturador e ajudou mas provavelmente ainda ta errado
        if (xa < -1*Ea) xa = -1*Ea;
        
        // Calcular a tensão de referência `vse_ref`
        double vl_ref = Vel_ref * sin(theta);
        double vse_ref = eg - vl_ref;
        
        // Comparação com onda triangular para determinar chaveamento
        double vsh_ref = xa + kp_res * ei1 + eg;
        //vsh_ref = eg;
        //if (vsh_ref > Ea) vsh_ref = Ea;
        //if (vsh_ref < -Ea) vsh_ref = -Ea;
        // Lógica de chaveamento utilizando `vtri` como entrada
        if (vsh_ref >= vtri) qsa = 1; else qsa = 0;
        if (vse_ref >= vtri) qla = 1; else qla = 0;

        // Saídas
        out[0] = qsa;
        out[1] = qla;
        out[2] = vsh_ref; // tensão de referência após controle ressonante
		out[3] = ig_ref;
        
    }
}

