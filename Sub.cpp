/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Sub.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/**********************************************************************/
/*                Definición funciones Menu                           */
/**********************************************************************/

Menu::Menu():tiempo_presionado(0),delay_presion(0),presion_boton1(nullptr),presion_boton2(nullptr),presion_boton3(nullptr),presion_boton4(nullptr),current_menu(0)
         ,estado_code(0),delay_code(0),developer_change(nullptr),developer_mode(nullptr),button_assignment(nullptr){}

void Menu::modify_bool_variable(bool& variable, const char* modify_message) {
    if (*presion_boton1 && (tiempo_presionado == 0)) {
        variable = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable ? "true" : "false");
    } else if (*presion_boton2 && (tiempo_presionado == 0)) {
        variable = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable ? "true" : "false");
    }
}

// Función para modificar variables numéricas (int, float, etc.)
template <typename T>
void Menu::modify_numeric_variable(T& variable, T min_value, T max_value, T step_size, const char* modify_message) {
    if (*presion_boton1) {
        if (tiempo_presionado == 0) {
            if (variable < max_value) {
                variable += step_size;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable);
        } else if (tiempo_presionado > 1000) {
            if (delay_presion > 50) {
                delay_presion = 0;
                if (variable < max_value) {
                    variable += step_size;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable);
                }
            } else {
                delay_presion += 1;
            }
        } else if (tiempo_presionado > 200) {
            if (delay_presion > 200) {
                delay_presion = 0;
                if (variable < max_value) {
                    variable += step_size;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable);
                }
            } else {
                delay_presion += 1;
            }
        }
    } else if (*presion_boton2) {
        if (tiempo_presionado == 0) {
            if (variable > min_value) {
                variable -= step_size;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable);
        } else if (tiempo_presionado > 1000) {
            if (delay_presion > 50) {
                delay_presion = 0;
                if (variable > min_value) {
                    variable -= step_size;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable);
                }
            } else {
                delay_presion += 1;
            }
        } else if (tiempo_presionado > 200) {
            if (delay_presion > 200) {
                delay_presion = 0;
                if (variable > min_value) {
                    variable -= step_size;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, modify_message, variable);
                }
            } else {
                delay_presion += 1;
            }
        }
    }
}

void Menu::run() {

    detect_code();

    if(*developer_mode){
        change_menu(); // Cambiar automáticamente entre los menús
        for (uint16_t i = 0; i < variable_menus.size(); ++i) {
         if (current_menu == i) {
            VariableMenuBase* ptr = variable_menus[i];

                if (ptr->tipe==2) {
                    FloatVariableMenu* floatVar = static_cast<FloatVariableMenu*>(ptr);
                    modify_numeric_variable(floatVar->variable, floatVar->min_value, floatVar->max_value, floatVar->step_size, floatVar->modify_message);
                } else if (ptr->tipe==1) {
                    IntVariableMenu* intVar = static_cast<IntVariableMenu*>(ptr);
                    modify_numeric_variable(intVar->variable, intVar->min_value, intVar->max_value, intVar->step_size, intVar->modify_message);
                } else if (ptr->tipe==0) {
                    BoolVariableMenu* boolVar = static_cast<BoolVariableMenu*>(ptr);
                    modify_bool_variable(boolVar->variable, boolVar->modify_message);
                }
            }
        }
    }else{
        if(*presion_boton3 && (tiempo_presionado == 400)){
            *button_assignment=(*button_assignment+1)%3;
            if(*button_assignment==0){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "/************************************************************/\n             Modo Costura\n/************************************************************/");
            }else if(*button_assignment==1){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "/************************************************************/\n             Modo Normal\n/************************************************************/");
            }else if(*button_assignment==2){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "/************************************************************/\n             Modo Ajuste de Ganancias\n/************************************************************/");
            }
        }else if(*presion_boton4 && (tiempo_presionado == 400)){
            *button_assignment=(*button_assignment-1+3)%3;
            if(*button_assignment==0){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "/************************************************************/\n             Modo Costura\n/************************************************************/");
            }else if(*button_assignment==1){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "/************************************************************/\n             Modo Normal\n/************************************************************/");
            }else if(*button_assignment==2){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "/************************************************************/\n             Modo Ajuste de Ganancias\n/************************************************************/");
            }
        }
    }

    // Reiniciar tiempo_presionado y delay_presion cuando no se presionan los botones
    if (!*presion_boton1 && !*presion_boton2 && !*presion_boton3 && !*presion_boton4) {
        tiempo_presionado = 0;
        delay_presion = 0;
    } else {
        if (tiempo_presionado < 1200) {
            tiempo_presionado += 1;
        }
    }
    if(delay_code>0){
        delay_code+=-1;
    }
}

void Menu::setVariables(bool& boton1, bool& boton2, bool& boton3, bool& boton4, bool& change_developer, bool& mode_developer, uint16_t& assignment_button) {
    presion_boton1 = &boton1;
    presion_boton2 = &boton2;
    presion_boton3 = &boton3;
    presion_boton4 = &boton4;
    developer_change = &change_developer;
    developer_mode = &mode_developer;
    button_assignment = &assignment_button;
}

void Menu::change_menu() {
    if (*presion_boton3 && (tiempo_presionado == 0)) {
        current_menu = (current_menu + 1) % variable_menus.size();
        const char* welcome_message = variable_menus[current_menu]->welcome_message;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", welcome_message);
    } else if (*presion_boton4 && (tiempo_presionado == 0)) {
        current_menu = (current_menu + variable_menus.size() - 1) % variable_menus.size();
        const char* welcome_message = variable_menus[current_menu]->welcome_message;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", welcome_message);
    }
}

// Función para agregar menús de modificación de variables
void Menu::add_bool_variable_menu(const char* welcome_message, const char* modify_message, bool& variable) {
    variable_menus.push_back(new BoolVariableMenu{welcome_message, modify_message, variable});
}

// Para variables de tipo int
void Menu::add_int_variable_menu(const char* welcome_message, const char* modify_message, int& variable, int min_value, int max_value, int step_size) {
    variable_menus.push_back(new IntVariableMenu{welcome_message, modify_message, variable, min_value, max_value, step_size});
}

// Para variables de tipo float
void Menu::add_float_variable_menu(const char* welcome_message, const char* modify_message, float& variable, float min_value, float max_value, float step_size) {
    variable_menus.push_back(new FloatVariableMenu{welcome_message, modify_message, variable, min_value, max_value, step_size});
}

void Menu::detect_code(){
    uint16_t boton=0;
    if(*presion_boton1){
        boton=1;
    }else if(*presion_boton2){
        boton=2;
    }else if(*presion_boton3){
        boton=3;
    }else if(*presion_boton4){
        boton=4;
    }

    if((estado_code==0)&&(boton==code[0])&&(tiempo_presionado == 0)){
        delay_code=400;
        estado_code=1;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cambio a estado 1");
    }else if((estado_code==1)&&(boton==code[1])&&(tiempo_presionado == 0)&&(delay_code>0)){
        delay_code=400;
        estado_code=2;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cambio a estado 2");
    }else if((estado_code==2)&&(boton==code[2])&&(tiempo_presionado == 0)&&(delay_code>0)){
        delay_code=400;
        estado_code=3;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cambio a estado 3");
    }else if((estado_code==3)&&(boton==code[3])&&(tiempo_presionado == 0)&&(delay_code>0)){
        delay_code=0;
        *developer_change=true;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Se ingresó clave");
    }else if(delay_code==0){
        estado_code=0;
    }else if((boton!=0)&&(boton!=code[estado_code])&&(tiempo_presionado == 0)){
        estado_code=0;
        delay_code=0;
    }

}
void Sub::mode_control(){
    if(sostenido1&&!presion_boton1){
        sostenido1=false;
        soltado1=true;
    }
    if(sostenido2&&!presion_boton2){
        sostenido2=false;
        soltado2=true;
    }
    if(sostenido3&&!presion_boton3){
        sostenido3=false;
        soltado3=true;
    }
    if(sostenido4&&!presion_boton4){
        sostenido4=false;
        soltado4=true;
    }
    if(sostenido5&&!presion_boton5){
        sostenido5=false;
        soltado5=true;
    }
    if(sostenido6&&!presion_boton6){
        sostenido6=false;
        soltado6=true;
    }
    if(developer_change){
        developer_change=false;
        if(developer_mode){
            developer_mode=false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Modo Operario");
        }else{
            developer_mode=true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Modo Developer");
        }
    }

    if(!developer_mode){
        if(button_assignment==0){
            abrir_garra_izquierda(presion_boton2,sostenido2,soltado2);
            abrir_garra_derecha(presion_boton1,sostenido1,soltado1);
            cerrar_garra(presion_boton5,sostenido5,soltado5);
        }else if(button_assignment==1){
            subir_camara(presion_boton6,sostenido6,soltado6);
            bajar_camara(presion_boton5,sostenido5,soltado5);
            aumentar_luz(presion_boton1,sostenido1,soltado1);
            disminuir_luz(presion_boton2,sostenido2,soltado2);
            aumentar_ganancia(presion_boton3,sostenido3,soltado3);
            disminuir_ganancia(presion_boton4,sostenido4,soltado4);
        }else if(button_assignment==2){
            aumentar_ganancia_rotacional(presion_boton6,sostenido6,soltado6);
            disminuir_ganancia_rotacional(presion_boton5,sostenido5,soltado5);
            aumentar_ganancia_horizontal(presion_boton1,sostenido1,soltado1);
            disminuir_ganancia_horizontal(presion_boton2,sostenido2,soltado2);
            aumentar_ganancia_vertical(presion_boton3,sostenido3,soltado3);
            disminuir_ganancia_vertical(presion_boton4,sostenido4,soltado4);
        }
    }
    if(soltado1)
        soltado1=false;
    if(sostenido1==false && presion_boton1)
        sostenido1=true;
    if(soltado2)
        soltado2=false;
    if(sostenido2==false && presion_boton2)
        sostenido2=true;
    if(soltado3)
        soltado3=false;
    if(sostenido3==false && presion_boton3)
        sostenido3=true;
    if(soltado4)
        soltado4=false;
    if(sostenido4==false && presion_boton4)
        sostenido4=true;
    if(soltado5)
        soltado5=false;
    if(sostenido5==false && presion_boton5)
        sostenido5=true;
    if(soltado6)
        soltado6=false;
    if(sostenido6==false && presion_boton6)
        sostenido6=true;
}


/*
  constructor for main Sub class
 */
Sub::Sub()
    : logger(g.log_bitmask),
          control_mode(MANUAL),
          motors(MAIN_LOOP_RATE),
          auto_mode(Auto_WP),
          guided_mode(Guided_WP),
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
          inertial_nav(ahrs),
          ahrs_view(ahrs, ROTATION_NONE),
          attitude_control(ahrs_view, aparm, motors, scheduler.get_loop_period_s()),
          pos_control(ahrs_view, inertial_nav, motors, attitude_control, scheduler.get_loop_period_s()),
          wp_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          loiter_nav(inertial_nav, ahrs_view, pos_control, attitude_control),
          circle_nav(inertial_nav, ahrs_view, pos_control),
          param_loader(var_info)
{
    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    failsafe.pilot_input = true;
#endif
}

Sub sub;
AP_Vehicle& vehicle = sub;
