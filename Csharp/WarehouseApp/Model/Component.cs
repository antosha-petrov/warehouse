namespace WarehouseApp.Model
{
    // Модель товара
    public class Component
    {
        // Название товара
        public string Name { get; set; }
        
        // Количество товара
        public int Quantity { get; set; }
        
        // Изображение товара
        public string ImageSource { get; set; }
        
        // Стелаж
        public int Rack { get; set; }
        
        // Ячейка
        public int Cell { get; set; }
        
        // Полка
        public int Shelf { get; set; }

        // Конструктор класса
        public Component(string name, string imageSource)
        {
            Name = name;
            Quantity = 0;
            Rack = 0;
            Cell = 0;
            Shelf = 0;
            ImageSource = imageSource;
        }
    }
}
