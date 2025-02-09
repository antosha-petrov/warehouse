namespace Models
{
    public class OrderItem
    {
        public string Id { get; private set; }
        
        public GoodsItem Goods { get; set; }

        public int Quantity { get; set; }

        public int Rack { get; set; }

        public int Cell { get; set; }

        public int Shelf { get; set; }

        public OrderItem(GoodsItem goods, int quantity)
        {
            Id = Guid.NewGuid().ToString();
            Goods = goods;
            Quantity = quantity;
            Rack = 0;
            Cell = 0;
            Shelf = 0;
        }
    }
}
