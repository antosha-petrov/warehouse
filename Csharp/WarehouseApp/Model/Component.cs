namespace WarehouseApp.Model
{
    public class Component
    {
        public string Name { get; set; }
        public int Quantity { get; set; }
        public int Rack { get; set; }
        public int Cell { get; set; }
        public int Shelf { get; set; }

        public Component(string name)
        {
            Name = name;
            Quantity = 0;
            Rack = 0;
            Cell = 0;
            Shelf = 0;
        }
    }
}
