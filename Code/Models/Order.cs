namespace Models
{
    public class Order
    {
        public string Id { get; set; }

        public string Status { get; set; }

        public List<OrderItem> Items { get; set; }

        public Order(List<OrderItem> items)
        {
            Id = Guid.NewGuid().ToString();
            Items = items;
            Status = "Pattern";
        }
    }
}
