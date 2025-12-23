
//--------defanes--------------------
#define MAX_DIM 0
#define MIN_DIM 200


typedef struct Board {
    uint8_t Adres; // Адрес платы
} board_t;
extern board_t Brd;






//--------------function
void isr();
void updateLight();